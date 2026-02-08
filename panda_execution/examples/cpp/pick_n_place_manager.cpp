/**
 * NODO: pick_n_place_manager
 * DESCRIZIONE: Orchestratore Fase 3.
 * AGGIORNAMENTO: Supporto per servizio custom MoveToPose con calcolo TF.
 */

#include <chrono>
#include <memory>
#include <thread>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/empty.hpp" // Libreria per i messaggi Empty
#include "std_msgs/msg/bool.hpp" // Header per leggere lo stato del monitor

// HEADER SERVIZIO CUSTOM
#include "panda_execution/srv/move_to_pose.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp" // Header per muovere le pinze

#include "offboard_rl/srv/drone_pose.hpp"

// HEADER TF2 (Per calcolare la discesa)
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class PickNPlaceManager : public rclcpp::Node {
public:
    using MoveToPoseSrv = panda_execution::srv::MoveToPose;

    PickNPlaceManager() : Node("pick_n_place_manager") {
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);


        // Inizializza stato grip a false per sicurezza
        latest_grip_state_ = false;

        // Publisher per il Gripper
        pub_gripper_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/gripper_trajectory_controller/joint_trajectory", 10);

        // --- NUOVO: Publisher per il Braccio (Arm Controller) ---
        pub_arm_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);

        //--- PUBLISHERS PER GAZEBO BRIDGE ---
        pub_attach_ = this->create_publisher<std_msgs::msg::Empty>("/panda/attach", 10);
        pub_detach_ = this->create_publisher<std_msgs::msg::Empty>("/panda/detach", 10);

        // NUOVO: Subscriber per feedback stato Presa (dal nodo Python)
        sub_grip_monitor_ = this->create_subscription<std_msgs::msg::Bool>(
            "/panda/current_grip_status", 
            10, 
            std::bind(&PickNPlaceManager::grip_monitor_callback, this, std::placeholders::_1));

        // --- INIZIALIZZAZIONE TF ---
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Server Principale
        server_ = this->create_service<std_srvs::srv::SetBool>(
            "execute_pick_n_place",
            std::bind(&PickNPlaceManager::handle_request, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_);

        // Client Trigger (Servo)
        client_start_servo_ = this->create_client<std_srvs::srv::Trigger>(
            "/servo_node/start_servo", rmw_qos_profile_services_default, callback_group_);

        client_stop_servo_ = this->create_client<std_srvs::srv::Trigger>(
            "/servo_node/stop_servo", rmw_qos_profile_services_default, callback_group_);

        // Client Visual Servo
        client_visual_servo_ = this->create_client<std_srvs::srv::SetBool>(
            "start_visual_servo", rmw_qos_profile_services_default, callback_group_);

        // --- CLIENT PICK TASK (Aggiornato a MoveToPose) ---
        client_pick_ = this->create_client<MoveToPoseSrv>(
            "start_panda_pick", rmw_qos_profile_services_default, callback_group_);

            // --- CLIENT MOVE TASK (Aggiornato a MoveToPose) ---
        client_move_ = this->create_client<MoveToPoseSrv>(
            "panda_move_task", rmw_qos_profile_services_default, callback_group_);

        client_get_drone_pose_ = this->create_client<offboard_rl::srv::DronePose>(
        "get_drone_pose_enu", rmw_qos_profile_services_default, callback_group_);
            
        is_task_running_ = false;

        RCLCPP_INFO(this->get_logger(), "=== PICK & PLACE MANAGER (MoveToPose Ready) ===");
    }

private:
    void handle_request(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        
        // 1. GESTIONE STOP
        if (!request->data) {
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                is_task_running_ = false; 
            }
            pub_detach_->publish(std_msgs::msg::Empty()); // Forza il rilascio in caso di stop
            response->success = false;
            response->message = "Stop ricevuto.";
            call_trigger(client_stop_servo_, "Stop Servo");
            return;
        }

        // 2. CONTROLLO RIENTRANZA
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (is_task_running_) {
                response->success = false;
                response->message = "BUSY: Sequenza in corso.";
                return;
            }
            is_task_running_ = true;
        }

        RCLCPP_INFO(this->get_logger(), ">>> AVVIO STEP 3 <<<");

        // --- FASE A: VISUAL SERVOING ---
        if (!call_trigger(client_start_servo_, "Start Servo")) {
            abort_sequence(response, "Errore attivazione Servo.");
            return;
        }

        bool servo_success = call_bool_service(client_visual_servo_, true, "Visual Servoing", 60s);
        call_trigger(client_stop_servo_, "Stop Servo"); // Stop sempre

        if (!servo_success) {
            abort_sequence(response, "Visual Servoing Fallito.");
            return;
        }

        std::this_thread::sleep_for(1s); // Stabilizzazione

        // --- FASE B: PICK TASK (MoveToPose) ---

        // ==========================================================================
        // FASE: PRE-PICK (CHIUSURA PINZE + ATTACH + FEEDBACK)
        // ==========================================================================
        RCLCPP_INFO(this->get_logger(), "Chiusura pinze a 0.04m...");              
        move_gripper(0.04);
        std::this_thread::sleep_for(3000ms); 

        RCLCPP_INFO(this->get_logger(), "Invio comando ATTACH...");
        pub_attach_->publish(std_msgs::msg::Empty());
        
        // WAIT FOR FEEDBACK (Aspetta fino a 3 secondi che diventi TRUE)
        if (!wait_for_grip_confirmation(true, 3000ms)) {
             abort_sequence(response, "ERRORE CRITICO: Attach fallito (nessun feedback dal plugin).");
             // Safety release
             move_gripper(0.07);
             return;
        }
        RCLCPP_INFO(this->get_logger(), ">> ATTACH CONFERMATO <<");
        
        // --- FASE B: PICK TASK (MoveToPose) ---

        RCLCPP_INFO(this->get_logger(), "Richiesta posizione drone al servizio...");
        // 1. Chiamata al servizio get_drone_pose_enu
        if (!client_get_drone_pose_->wait_for_service(2s)) {
            abort_sequence(response, "Servizio get_drone_pose_enu non disponibile!");
            return;
        }
        auto pose_req = std::make_shared<offboard_rl::srv::DronePose::Request>();
        auto pose_future = client_get_drone_pose_->async_send_request(pose_req);
        if (pose_future.wait_for(3s) == std::future_status::timeout) {
        abort_sequence(response, "Timeout durante il recupero posa drone.");
        return;
        }   


        auto pose_res = pose_future.get();
        if (!pose_res->success) {
            abort_sequence(response, "Il monitor PX4 ha restituito errore nel recupero posa.");
            return;
        }
        double target_x = pose_res->pose.position.x + 0.45;
        double target_y = pose_res->pose.position.y;
        double target_z = pose_res->pose.position.z + 0.2;
                
        RCLCPP_INFO(this->get_logger(), "Target calcolato: X=%.2f, Y=%.2f, Z=%.2f", target_x, target_y, target_z);

        // 3. Chiamata al servizio MoveToPose con coordinate dinamiche
        bool pick_success = call_pick_service(
            target_x, target_y, target_z, 
            -3.14, 0.7, 0.0,   // Orientamento costante richiesto
            false,            // Cartesian Mode
            10s
        );

        std::this_thread::sleep_for(1s);
        // ========================================================
        // FASE: POST-PICK (DETACH + FEEDBACK + APERTURA)
        // ========================================================
        if (pick_success) {
            RCLCPP_INFO(this->get_logger(), "Pick completato. Invio DETACH...");
            pub_detach_->publish(std_msgs::msg::Empty());
            
            // WAIT FOR FEEDBACK (Aspetta fino a 3 secondi che diventi FALSE)
            if (!wait_for_grip_confirmation(false, 3000ms)) {
                RCLCPP_WARN(this->get_logger(), "ATTENZIONE: Detach non confermato, forzo apertura pinze.");
            } else {
                RCLCPP_INFO(this->get_logger(), ">> DETACH CONFERMATO <<");
            }

            RCLCPP_INFO(this->get_logger(), "Apertura pinze a 0.07m...");
            move_gripper(0.07);
            // Attende che le pinze si aprano prima di muovere il braccio
            std::this_thread::sleep_for(1000ms);
            
            // --- QUI INSERIAMO IL TUO COMANDO PER TORNARE "HOME" ---
            RCLCPP_INFO(this->get_logger(), "Ritorno in posizione HOME...");
            
            // 3. Chiamiamo il servizio MoveToPose
            bool move_home_success = call_move_service(
                1.3, 0.0, 0.5,         // IN FUTURO QUESTA PRENDERÀ LA POSIZIONE DEL DRONE 
                -3.14, 0.0, 0.0,   // L'ORIENTAMENTO SARÀ COSTANTE
                false,                           // Cartesian Mode = TRUE (linea retta)
                10s
            );

            if(move_home_success){
                RCLCPP_INFO(this->get_logger(), "Home Move completato. Passaggio al prossimo Step");
                response->success = true;
                response->message = "Sequenza Completata.";
                RCLCPP_INFO(this->get_logger(), ">>> SUCCESS <<<");
            }
            else{
                abort_sequence(response, "Home Move Task Fallito.");
            }
                
        } else {
            // Safety
            pub_detach_->publish(std_msgs::msg::Empty());
            move_gripper(0.07);
            abort_sequence(response, "Pick Task Fallito.");
            return;
        }

        // SUCCESSO
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            is_task_running_ = false; 
        }

        
    }

    // --- NUOVO HELPER PER MoveToPose ---
    bool call_pick_service(double x, double y, double z, double r, double p, double yw, bool cartesian, std::chrono::seconds timeout) {
        if (!client_pick_->wait_for_service(2s)) {
            RCLCPP_ERROR(this->get_logger(), "Servizio Pick non disponibile.");
            return false;
        }

        auto req = std::make_shared<MoveToPoseSrv::Request>();
        req->x = x; req->y = y; req->z = z;
        req->roll = r; req->pitch = p; req->yaw = yw;
        req->cartesian_mode = cartesian;

        RCLCPP_INFO(this->get_logger(), "Inviando Pick Target: Z=%.3f (Cartesian=%d)", z, cartesian);

        auto future = client_pick_->async_send_request(req);
        if (future.wait_for(timeout) == std::future_status::timeout) {
             RCLCPP_ERROR(this->get_logger(), "TIMEOUT Pick Task");
             return false;
        }
        
        auto res = future.get();
        if (!res->success) RCLCPP_WARN(this->get_logger(), "Pick Server Error: %s", res->message.c_str());
        return res->success;
    }

    bool call_move_service(double x, double y, double z, double r, double p, double yw, bool cartesian, std::chrono::seconds timeout) {
        if (!client_move_->wait_for_service(2s)) {
            RCLCPP_ERROR(this->get_logger(), "Servizio Pick non disponibile.");
            return false;
        }

        auto req = std::make_shared<MoveToPoseSrv::Request>();
        req->x = x; req->y = y; req->z = z;
        req->roll = r; req->pitch = p; req->yaw = yw;
        req->cartesian_mode = cartesian;

        RCLCPP_INFO(this->get_logger(), "Inviando Pick Target: Z=%.3f (Cartesian=%d)", z, cartesian);

        auto future = client_move_->async_send_request(req);
        if (future.wait_for(timeout) == std::future_status::timeout) {
             RCLCPP_ERROR(this->get_logger(), "TIMEOUT Pick Task");
             return false;
        }
        
        auto res = future.get();
        if (!res->success) RCLCPP_WARN(this->get_logger(), "Pick Server Error: %s", res->message.c_str());
        return res->success;
    }
    

    // Helper per comandare l'apertura/chiusura delle pinze
    void move_gripper(double width) {
        trajectory_msgs::msg::JointTrajectory msg;
        msg.joint_names = {"panda_finger_joint1", "panda_finger_joint2"};
        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {width, width}; // La distanza totale è divisa tra le due dita
        point.time_from_start = rclcpp::Duration::from_seconds(2.0);
        
        msg.points.push_back(point);
        pub_gripper_->publish(msg);
    }

    // --- NUOVO HELPER PER TF ---
    bool get_current_pose(double &x, double &y, double &z, double &r, double &p, double &yw) {
        try {
            // Cerca la trasformata da 'world' a 'panda_hand_tcp'
            geometry_msgs::msg::TransformStamped t = 
                tf_buffer_->lookupTransform("world", "panda_hand_tcp", tf2::TimePointZero);
            
            x = t.transform.translation.x;
            y = t.transform.translation.y;
            z = t.transform.translation.z;

            tf2::Quaternion q(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w);
            
            tf2::Matrix3x3(q).getRPY(r, p, yw);
            return true;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "TF Exception: %s", ex.what());
            return false;
        }
    }

    // Helper abort
    void abort_sequence(std::shared_ptr<std_srvs::srv::SetBool::Response> response, std::string error_msg) {
        RCLCPP_ERROR(this->get_logger(), "ABORT: %s", error_msg.c_str());
        response->success = false;
        response->message = error_msg;
        std::lock_guard<std::mutex> lock(state_mutex_);
        is_task_running_ = false;
    }

    // Helper Trigger
    bool call_trigger(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client, std::string name) {
        if (!client->wait_for_service(2s)) return false;
        auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        if (future.wait_for(3s) == std::future_status::timeout) return false;
        return future.get()->success;
    }

    // Helper SetBool (Per Visual Servo)
    bool call_bool_service(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client, bool data, std::string name, std::chrono::seconds timeout) {
        if (!client->wait_for_service(2s)) return false;
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = data;
        auto future = client->async_send_request(req);
        if (future.wait_for(timeout) == std::future_status::timeout) return false;
        return future.get()->success;
    }
    // --- CALLBACK PER MONITORAGGIO STATO ---
    void grip_monitor_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        latest_grip_state_.store(msg->data);
    }

    // --- NUOVA FUNZIONE DI ATTESA ATTIVA ---
    bool wait_for_grip_confirmation(bool expected_state, std::chrono::milliseconds timeout) {
        auto start_time = std::chrono::steady_clock::now();
        rclcpp::Rate rate(10); // Check a 10Hz

        RCLCPP_INFO(this->get_logger(), "In attesa di conferma %s...", expected_state ? "ATTACH" : "DETACH");

        while (rclcpp::ok()) {
            if (latest_grip_state_.load() == expected_state) {
                return true; // Stato confermato!
            }

            if ((std::chrono::steady_clock::now() - start_time) > timeout) {
                RCLCPP_WARN(this->get_logger(), "TIMEOUT: Conferma grip non ricevuta!");
                return false;
            }
            rate.sleep();
        }
        return false;
    }
    void move_arm_home(double j1, double j2, double j3, double j4, double j5, double j6, double j7) {
            trajectory_msgs::msg::JointTrajectory msg;
            msg.joint_names = {"panda_joint1", "panda_joint2", "panda_joint3", 
                            "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
            
            trajectory_msgs::msg::JointTrajectoryPoint point;
            // Valori presi dal tuo comando CLI
            point.positions = {j1,-j2, j3, j4,j5, j6, j7};
            point.time_from_start = rclcpp::Duration::from_seconds(2.0); // Ci mette 2 secondi
            
            msg.points.push_back(point);
            pub_arm_->publish(msg);
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr server_;
    
    // Clients
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_start_servo_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_stop_servo_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_visual_servo_;
    
    // Client modificato per il tipo MoveToPose
    rclcpp::Client<MoveToPoseSrv>::SharedPtr client_pick_;
    rclcpp::Client<MoveToPoseSrv>::SharedPtr client_move_;

    // TF
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_attach_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_detach_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_gripper_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_;

    // NUOVO: Subscriber per il monitoraggio
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_grip_monitor_;

    rclcpp::Client<offboard_rl::srv::DronePose>::SharedPtr client_get_drone_pose_;
    
    // NUOVO: Variabile atomica per lo stato
    std::atomic<bool> latest_grip_state_;

    
    std::mutex state_mutex_;
    bool is_task_running_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickNPlaceManager>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}