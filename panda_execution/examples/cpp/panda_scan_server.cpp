/**
 * NODO: panda_scan_server
 * VERSIONE: 7.0 (Active Monitoring + Smart Direction)
 * DESCRIZIONE:
 * 1. Controllo costante del marker (anche durante il posizionamento).
 * 2. Scansione bidirezionale (sceglie il lato più vicino).
 * 3. Stop immediato basato sulla posizione REALE letta dai sensori.
 */

#include <chrono>
#include <memory>
#include <thread>
#include <cmath>
#include <string>
#include <vector>
#include <atomic>
#include <algorithm> // per std::abs

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp" 
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class PandaScanServer : public rclcpp::Node {
public:
    PandaScanServer() : Node("panda_scan_server") {
        cb_group_sub_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_srv_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        pub_arm_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);

        pub_gripper_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/gripper_trajectory_controller/joint_trajectory", 10);

        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = cb_group_sub_;

        // Subscriber Lista Marker
        sub_aruco_list_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
            "/aruco_marker_publisher/markers_list", 10, 
            std::bind(&PandaScanServer::aruco_list_callback, this, std::placeholders::_1), sub_opt);

        // Subscriber Joint States
        sub_joints_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&PandaScanServer::joint_state_callback, this, std::placeholders::_1), sub_opt);

        service_ = this->create_service<std_srvs::srv::SetBool>(
            "scan_for_marker",
            std::bind(&PandaScanServer::handle_scan, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, cb_group_srv_);

        current_joint_positions_.resize(7, 0.0);
        marker_detected_.store(false);

        RCLCPP_INFO(this->get_logger(), "Scan Server 7.0 (Bidirezionale + Active Check) Pronto.");
    }

private:
    std::atomic<bool> marker_detected_;
    bool joint_state_received_ = false;
    std::vector<double> current_joint_positions_;
    
    // Limiti scansione
    const double LIMIT_NEG = -2.80; 
    const double LIMIT_POS = 2.80;  
    const double STEP_RAD = 0.05; 
    const double GRIPPER_OPEN_VAL = 0.07; 

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_gripper_;
    rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr sub_aruco_list_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joints_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    rclcpp::CallbackGroup::SharedPtr cb_group_sub_;
    rclcpp::CallbackGroup::SharedPtr cb_group_srv_;

    void aruco_list_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        if (!msg->data.empty()) {
            if (!marker_detected_.load()) {
                RCLCPP_WARN(this->get_logger(), ">>> MARKER RILEVATO! (IDs: %zu) <<<", msg->data.size());
                marker_detected_.store(true);
            }
        }
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "panda_joint1") current_joint_positions_[0] = msg->position[i];
            else if (msg->name[i] == "panda_joint2") current_joint_positions_[1] = msg->position[i];
            else if (msg->name[i] == "panda_joint3") current_joint_positions_[2] = msg->position[i];
            else if (msg->name[i] == "panda_joint4") current_joint_positions_[3] = msg->position[i];
            else if (msg->name[i] == "panda_joint5") current_joint_positions_[4] = msg->position[i];
            else if (msg->name[i] == "panda_joint6") current_joint_positions_[5] = msg->position[i];
            else if (msg->name[i] == "panda_joint7") current_joint_positions_[6] = msg->position[i];
        }
        joint_state_received_ = true;
    }

    // --- FUNZIONE CRITICA: ATTESA ATTIVA ---
    // Sostituisce sleep_for. Controlla il marker ogni 10ms.
    // Ritorna TRUE se il marker è stato trovato durante l'attesa.
    bool wait_and_check(double seconds) {
        int steps = static_cast<int>(seconds * 100); // step da 10ms
        for (int i = 0; i < steps; ++i) {
            if (marker_detected_.load()) {
                perform_emergency_stop();
                return true; // Trovato! Interrompi tutto.
            }
            std::this_thread::sleep_for(10ms);
        }
        return false;
    }

    void perform_emergency_stop() {
        // Leggiamo la posizione REALE attuale
        std::vector<double> stop_pose = current_joint_positions_;
        double current_j1 = stop_pose[0];
        
        RCLCPP_WARN(this->get_logger(), "STOP IMMEDIATO a J1 = %.3f", current_j1);
        
        // Invio comando stop multiplo per sicurezza
        for(int k=0; k<5; k++) {
            // duration = 0.5s è un buon compromesso per fermare il controller
            publish_arm_command(current_j1, stop_pose, 0.5); 
            std::this_thread::sleep_for(10ms);
        }
    }

    void handle_scan(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                     std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        
        if (!request->data) {
            response->success = false;
            response->message = "Annullato.";
            return;
        }

        if (!joint_state_received_) {
            response->success = false;
            response->message = "ERRORE: joint_states mancante.";
            return;
        }

        // 0. RESET INIZIALE
        marker_detected_.store(false);
        std::vector<double> locked_pose = current_joint_positions_; // Foto della posa iniziale

        // 1. APERTURA GRIPPER (con controllo attivo)
        RCLCPP_INFO(this->get_logger(), "1. Apertura Gripper...");
        open_gripper();
        // Aspettiamo 1.5 secondi controllando se appare un marker
        if (wait_and_check(1.5)) {
            response->success = true;
            response->message = "Marker trovato durante apertura pinza.";
            return;
        }

        // 2. DECISIONE STRATEGIA DI SCANSIONE (Bidirezionale)
        double start_j1 = current_joint_positions_[0];
        double target_start_scan; // Dove inizia la scansione vera e propria
        double target_end_scan;   // Dove finisce
        double scan_step;

        // Se siamo più vicini al limite Positivo (+), scansioniamo verso il Negativo (-)
        // Se siamo più vicini al limite Negativo (-), scansioniamo verso il Positivo (+)
        if (std::abs(start_j1 - LIMIT_POS) < std::abs(start_j1 - LIMIT_NEG)) {
            RCLCPP_INFO(this->get_logger(), "Logica: Da POSITIVO a NEGATIVO");
            target_start_scan = LIMIT_POS;
            target_end_scan = LIMIT_NEG;
            scan_step = -STEP_RAD; // Passo negativo
        } else {
            RCLCPP_INFO(this->get_logger(), "Logica: Da NEGATIVO a POSITIVO");
            target_start_scan = LIMIT_NEG;
            target_end_scan = LIMIT_POS;
            scan_step = STEP_RAD;  // Passo positivo
        }

        // 3. POSIZIONAMENTO AL PUNTO DI START (con controllo attivo)
        RCLCPP_INFO(this->get_logger(), "2. Spostamento al punto di inizio (%.2f)...", target_start_scan);
        publish_arm_command(target_start_scan, locked_pose, 2.0); // 2 secondi per arrivarci
        
        // Durante il movimento verso l'inizio, controlliamo se vediamo qualcosa
        if (wait_and_check(2.0)) {
            response->success = true;
            response->message = "Marker trovato mentre andavo in posizione di start.";
            return;
        }

        // 4. ESECUZIONE SCANSIONE
        RCLCPP_INFO(this->get_logger(), "3. Scansione Loop...");
        
        double angle = target_start_scan;
        rclcpp::Rate loop_rate(20); // 20 Hz per maggior fluidità

        // Loop condizionale che gestisce sia incremento che decremento
        bool scanning = true;
        while (rclcpp::ok() && scanning) {
            
            // Check Immediato
            if (marker_detected_.load()) {
                perform_emergency_stop();
                response->success = true;
                response->message = "Marker trovato a " + std::to_string(angle);
                return;
            }

            // Movimento
            publish_arm_command(angle, locked_pose, 0.1);
            angle += scan_step;

            // Condizione di uscita dal loop (abbiamo superato il target?)
            if (scan_step > 0 && angle > target_end_scan) scanning = false;
            else if (scan_step < 0 && angle < target_end_scan) scanning = false;

            loop_rate.sleep();
        }

        // Fine scansione
        response->success = false;
        response->message = "Scansione completata. Nessun marker.";
        RCLCPP_INFO(this->get_logger(), "Fine corsa. Nessun marker.");
    }

    void open_gripper() {
        trajectory_msgs::msg::JointTrajectory msg;
        msg.joint_names = {"panda_finger_joint1", "panda_finger_joint2"};
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {GRIPPER_OPEN_VAL, GRIPPER_OPEN_VAL}; 
        point.time_from_start = rclcpp::Duration::from_seconds(1.0);
        msg.points.push_back(point);
        pub_gripper_->publish(msg);
    }

    void publish_arm_command(double joint1_angle, const std::vector<double>& locked_pose, double duration_sec) {
        trajectory_msgs::msg::JointTrajectory msg;
        msg.joint_names = {"panda_joint1", "panda_joint2", "panda_joint3", 
                           "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {
            joint1_angle,    
            locked_pose[1], locked_pose[2], locked_pose[3],
            locked_pose[4], locked_pose[5], locked_pose[6]
        };
        point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
        msg.points.push_back(point);
        pub_arm_->publish(msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PandaScanServer>();
    rclcpp::executors::MultiThreadedExecutor executor; 
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}