#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

// Header dei servizi
#include "offboard_rl/srv/check_at_base.hpp" // Task 1
#include "std_srvs/srv/set_bool.hpp"         // Task 2, 3 & 5

using namespace std::chrono_literals;

// Definizione degli stati del processo
enum class ProcessState {
    INITIALIZING,
    WAITING_FOR_SERVICES,
    DRONE_POSE_REQUEST,    // Step 1: Verifica presenza drone
    ARUCO_ROS_RESEARCH,    // Step 2: Rotazione Panda e ricerca ArUco
    PICK_AND_PLACE,         // Step 3: Visual Servoing + Picking
    DRONE_SHIPPING,         // Step 5: Missione Drone (Volo e Ritorno)
    FINISHED,               // Stato finale: Missione completata (Non usato nel loop continuo)
    IDLE
};

class NodeManager : public rclcpp::Node {
public:
    NodeManager() : Node("node_manager"), state_(ProcessState::INITIALIZING) {
        // Parametri configurabili
        this->declare_parameter<std::string>("reference_frame", "world");

        // 1. Inizializzazione TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 2. Client per Step 1 (Drone Monitor)
        check_base_client_ = this->create_client<offboard_rl::srv::CheckAtBase>("/check_if_at_base");

        // 3. Client per Step 2 (Panda Scan Server)
        scan_client_ = this->create_client<std_srvs::srv::SetBool>("/scan_for_marker");

        // 4. Client per Step 3 (Pick 'n Place Manager)
        pick_place_client_ = this->create_client<std_srvs::srv::SetBool>("/execute_pick_n_place");

        // 5. Client per Step 5 (Trajectory Manager - NUOVO)
        drone_mission_client_ = this->create_client<std_srvs::srv::SetBool>("/start_drone_mission");

        // 6. Timer di controllo principale (10Hz)
        timer_ = this->create_wall_timer(
            100ms, std::bind(&NodeManager::control_cycle, this));

        RCLCPP_INFO(this->get_logger(), "Node Manager avviato. Stato: INITIALIZING");
    }

private:
    // Variabili di stato
    ProcessState state_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // TF
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    // Clients
    rclcpp::Client<offboard_rl::srv::CheckAtBase>::SharedPtr check_base_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr scan_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr pick_place_client_; 
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr drone_mission_client_; // NUOVO

    // Dati Posa Panda
    double panda_x_ = 0.0;
    double panda_y_ = 0.0;
    double panda_z_ = 0.0;
    bool panda_pose_found_ = false;

    // Flag per gestire chiamate asincrone lunghe
    bool scan_in_progress_ = false; 
    bool pick_place_in_progress_ = false;
    bool step_5_in_progress_ = false; // NUOVO FLAG

    // --- CICLO DI CONTROLLO PRINCIPALE ---
    void control_cycle() {
        switch (state_) {
            case ProcessState::INITIALIZING:
                state_ = ProcessState::WAITING_FOR_SERVICES;
                break;

            case ProcessState::WAITING_FOR_SERVICES:
                if (check_services_availability()) {
                    RCLCPP_INFO(this->get_logger(), "Tutti i servizi sono ONLINE. Avvio Step 1.");
                    state_ = ProcessState::DRONE_POSE_REQUEST;
                }
                break;

            case ProcessState::DRONE_POSE_REQUEST:
                // Step 1: Continuo a chiedere finché il drone non è in base
                perform_check_at_base_task();
                break;

            case ProcessState::ARUCO_ROS_RESEARCH:
                // Step 2: Avvio la scansione
                if (!scan_in_progress_) {
                    perform_scan_task();
                }
                break;

            case ProcessState::PICK_AND_PLACE:
                // Step 3: Avvio Visual Servoing + Picking
                if (!pick_place_in_progress_) {
                    perform_pick_place_task();
                }
                break;
            
            // --- NUOVO CASE STEP 5 ---
            case ProcessState::DRONE_SHIPPING:
                if (!step_5_in_progress_) {
                    perform_step_5_logic();
                }
                break;

            case ProcessState::FINISHED:
                RCLCPP_INFO_ONCE(this->get_logger(), "================ MISSIONE FINITA ================");
                timer_->cancel();
                break;

            case ProcessState::IDLE:
                break;
        }
    }

    // --- FUNZIONI DI SUPPORTO ---

    bool check_services_availability() {
        bool s1 = check_base_client_->wait_for_service(500ms);
        bool s2 = scan_client_->wait_for_service(500ms);
        bool s3 = pick_place_client_->wait_for_service(500ms); 
        bool s4 = drone_mission_client_->wait_for_service(500ms); // Check NUOVO servizio
        
        if (!s1) RCLCPP_WARN(this->get_logger(), "Waiting for /check_if_at_base...");
        if (!s2) RCLCPP_WARN(this->get_logger(), "Waiting for /scan_for_marker...");
        if (!s3) RCLCPP_WARN(this->get_logger(), "Waiting for /execute_pick_n_place...");
        if (!s4) RCLCPP_WARN(this->get_logger(), "Waiting for /start_drone_mission...");

        return s1 && s2 && s3 && s4;
    }

    // ---------------------------------------------------------
    // STEP 1: LOGICA DRONE AT BASE
    // ---------------------------------------------------------
    void perform_check_at_base_task() {
        std::string target_frame = "panda_link0";
        std::string reference_frame;
        this->get_parameter("reference_frame", reference_frame);

        try {
            auto t = tf_buffer_->lookupTransform(reference_frame, target_frame, tf2::TimePointZero);
            panda_x_ = t.transform.translation.x;
            panda_y_ = t.transform.translation.y;
            panda_z_ = t.transform.translation.z;
            panda_pose_found_ = true;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "TF non trovato (%s -> %s): %s", reference_frame.c_str(), target_frame.c_str(), ex.what());
            return;
        }

        if (panda_pose_found_) {
            auto request = std::make_shared<offboard_rl::srv::CheckAtBase::Request>();
            request->panda_x = panda_x_;
            request->panda_y = panda_y_;
            request->panda_z = panda_z_;

            auto callback = [this](rclcpp::Client<offboard_rl::srv::CheckAtBase>::SharedFuture future) {
                try {
                    auto result = future.get();
                    if (result->is_at_base) {
                        RCLCPP_INFO(this->get_logger(), "STEP 1 COMPLETATO: Drone in base. Avvio Step 2 (Scan).");
                        state_ = ProcessState::ARUCO_ROS_RESEARCH;
                    } else {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                            "STEP 1 WAIT: %s (Dist: %.2fm)", result->message.c_str(), result->distance_from_panda);
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Service Call Failed: %s", e.what());
                }
            };
            check_base_client_->async_send_request(request, callback);
        }
    }

    // ---------------------------------------------------------
    // STEP 2: LOGICA SCANSIONE (Panda Scan Server)
    // ---------------------------------------------------------
    void perform_scan_task() {
        if (scan_in_progress_) return;

        RCLCPP_INFO(this->get_logger(), "STEP 2: Invio comando START Scansione...");
        scan_in_progress_ = true; 

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;

        auto callback = [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            try {
                auto result = future.get();
                
                if (result->success) {
                    RCLCPP_INFO(this->get_logger(), "STEP 2 SUCCESSO: Marker trovato! Stop Scanner.");

                    // STOP del server di scansione
                    auto stop_request = std::make_shared<std_srvs::srv::SetBool::Request>();
                    stop_request->data = false;
                    auto stop_callback = [](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture){}; 
                    scan_client_->async_send_request(stop_request, stop_callback);

                    scan_in_progress_ = false;
                    state_ = ProcessState::PICK_AND_PLACE; // Passaggio allo Step 3

                } else {
                    RCLCPP_WARN(this->get_logger(), "STEP 2: Fine corsa senza marker. Riprovo...");
                    scan_in_progress_ = false;
                }

            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Scan Call Failed: %s", e.what());
                scan_in_progress_ = false; 
            }
        };

        scan_client_->async_send_request(request, callback);
    }

    // ---------------------------------------------------------
    // STEP 3: LOGICA PICK 'N PLACE
    // ---------------------------------------------------------
    void perform_pick_place_task() {
        if (pick_place_in_progress_) return;

        RCLCPP_INFO(this->get_logger(), "STEP 3: Invio comando al Pick'n Place Manager...");
        pick_place_in_progress_ = true;

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true; // START

        auto callback = [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            try {
                auto result = future.get();
                
                if (result->success) {
                    RCLCPP_INFO(this->get_logger(), "STEP 3 SUCCESSO: %s", result->message.c_str());
                    pick_place_in_progress_ = false;
                    
                    // --- MODIFICA: Transizione verso STEP 5 invece che FINISHED ---
                    state_ = ProcessState::DRONE_SHIPPING; 
                } else {
                    RCLCPP_ERROR(this->get_logger(), "STEP 3 FALLITO: %s. Riprovo.", result->message.c_str());
                    pick_place_in_progress_ = false;
                }

            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Pick'n Place Call Failed: %s", e.what());
                pick_place_in_progress_ = false;
            }
        };

        pick_place_client_->async_send_request(request, callback);
    }

    // ---------------------------------------------------------
    // STEP 5: MISSIONE DRONE (Con Loop a Step 1)
    // ---------------------------------------------------------
    void perform_step_5_logic() {
        if (step_5_in_progress_) return;

        RCLCPP_INFO(this->get_logger(), "STEP 5: Invio comando START al TrajectoryManager (Drone in volo)...");
        step_5_in_progress_ = true;

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true; // START Missione Drone

        auto callback = [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            try {
                auto result = future.get();
                
                if (result->success) {
                    // Se siamo qui, il drone è atterrato (il servizio è tornato True)
                    RCLCPP_INFO(this->get_logger(), "STEP 5 COMPLETATO: Drone missione finita. RITORNO A STEP 1.");
                    
                    step_5_in_progress_ = false;
                    // --- LOOP: Ritorna allo stato iniziale ---
                    state_ = ProcessState::DRONE_POSE_REQUEST; 

                } else {
                    RCLCPP_ERROR(this->get_logger(), "STEP 5 FALLITO: %s", result->message.c_str());
                    // In caso di fallimento logico, decidiamo di resettare il flag e riprovare
                    step_5_in_progress_ = false;
                }

            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Drone Mission Service Call Failed: %s", e.what());
                step_5_in_progress_ = false;
            }
        };

        // Chiamata asincrona (ma il server è bloccante, quindi il callback arriverà solo alla fine)
        drone_mission_client_->async_send_request(request, callback);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}