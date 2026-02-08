#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp> // Header per il servizio Bool
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <cmath>
#include <vector>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

enum class State { WAITING_START, IDLE, TAKEOFF, MOVING, LANDING, FINISHED };

class TrajectoryManager : public rclcpp::Node {
public:
    TrajectoryManager() : Node("trajectory_manager") {
        // Creiamo un gruppo di callback Reentrant per permettere il parallelismo
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Publisher per i goal
        goal_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/go_to_point/goal", 10);

        // --- SERVER: Sostituisce la subscription ---
        // Il servizio accetta una richiesta Bool e risponde quando la missione è finita
        service_server_ = this->create_service<std_srvs::srv::SetBool>(
            "start_drone_mission",
            std::bind(&TrajectoryManager::handle_service_request, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_); // Importante: usa il gruppo reentrant

        // Subscriber Posizione (Best Effort per PX4)
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        pos_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos_profile, 
            std::bind(&TrajectoryManager::position_callback, this, std::placeholders::_1));

        // Timer di controllo (Gira in parallelo al servizio)
        timer_ = this->create_wall_timer(
            200ms, 
            std::bind(&TrajectoryManager::control_loop, this), 
            callback_group_);
        
        state_ = State::WAITING_START;
        RCLCPP_INFO(this->get_logger(), "Manager PRONTO. In attesa di richiesta sul servizio: /start_drone_mission");
    }

private:
    // --- CALLBACK DEL SERVIZIO (BLOCCANTE) ---
    void handle_service_request(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) 
    {
        if (request->data) { // Se il messaggio è True
            RCLCPP_INFO(this->get_logger(), ">>> RICHIESTA RICEVUTA. INIZIO MISSIONE... <<<");
            
            // 1. Resetta e Avvia
            reset_mission();
            state_ = State::IDLE; // Questo sblocca il control_loop

            // 2. ATTESA BLOCCANTE (Wait Loop)
            // Aspettiamo qui finché il control_loop non imposta lo stato su FINISHED.
            rclcpp::Rate wait_rate(2); // Controllo 2 volte al secondo
            while (rclcpp::ok() && state_ != State::FINISHED) {
                // Durante questo loop, il control_loop continua a girare grazie al MultiThreadedExecutor
                wait_rate.sleep();
            }

            // 3. Risposta al Client
            response->success = true;
            response->message = "Missione Completata (Atterrato).";
            RCLCPP_INFO(this->get_logger(), ">>> MISSIONE FINITA. RISPONDO AL CLIENT. <<<");
        } else {
            RCLCPP_WARN(this->get_logger(), "Ricevuto False. Nessuna azione.");
            response->success = false;
            response->message = "Ricevuto False, missione abortita.";
        }
    }

    void reset_mission() {
        // Reset variabili per una nuova esecuzione
        pos_received_ = false;
        // Se necessario, resetta altri flag
    }

    void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        current_pos_ = {msg->x, msg->y, msg->z};
        pos_received_ = true;
    }

    void send_goal(double x, double y, double z, double yaw, double t) {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = {x, y, z, yaw, t};
        goal_publisher_->publish(msg);
        
        target_pos_ = {x, y, -z}; // Nota: Z invertita per coordinate locali NED/ENU
        
        if(t == -1.0) 
            RCLCPP_INFO(this->get_logger(), "cmd: ATTERRAGGIO");
        else
            RCLCPP_INFO(this->get_logger(), "cmd: GOAL [%.1f, %.1f, %.1f]", x, y, z);
    }

    double get_distance() {
        return std::sqrt(std::pow(current_pos_[0] - target_pos_[0], 2) +
                         std::pow(current_pos_[1] - target_pos_[1], 2) +
                         std::pow(current_pos_[2] - target_pos_[2], 2));
    }

    // --- LOOP DI CONTROLLO (Gira in background) ---
    void control_loop() {
        if (!pos_received_) return;

        double dist = get_distance();
        
        switch (state_) {
            case State::WAITING_START:
                // Non fa nulla finché il servizio non cambia lo stato in IDLE
                break;

            case State::IDLE:
                send_goal(0.0, 0.0, 10.0, 0.0, 5.0); // Decollo a 10m
                state_ = State::TAKEOFF;
                break;

            case State::TAKEOFF:
                if (dist < 0.7) {
                    RCLCPP_INFO(this->get_logger(), "Decollo completato -> Verso destinazione");
                    send_goal(10.5, 10.5, 7.5, 0.0, 10.0);
                    state_ = State::MOVING;
                }
                break;

            case State::MOVING:
                if (dist < 0.8) {
                    RCLCPP_INFO(this->get_logger(), "Destinazione raggiunta -> Atterraggio");
                    send_goal(0.0, 0.0, 0.0, 0.0, -1.0); // Comando Land
                    state_ = State::LANDING;
                }
                break;

            case State::LANDING:
                // Logica per determinare se è atterrato
                // In NED, l'altitudine è negativa (-10m = 10m altezza). Suolo = 0.
                // Controlliamo se la Z assoluta è vicina a zero (es. < 0.3m)
                {
                    double current_alt = std::abs(current_pos_[2]); // Assumendo Z=0 al suolo
                    
                    if (current_alt < 0.3) {
                         RCLCPP_INFO(this->get_logger(), "ATTERRAGGIO CONFERMATO (Suolo rilevato).");
                         state_ = State::FINISHED; // Questo sbloccherà il servizio!
                    } else {
                        // Re-invia comando landing ogni tanto per sicurezza
                        static int retry = 0;
                        if (retry++ % 10 == 0) send_goal(0.0, 0.0, 0.0, 0.0, -1.0);
                    }
                }
                break;

            case State::FINISHED:
                // Rimane qui finché non arriva una nuova richiesta di servizio
                break;
        }
    }

    // Variabili membro
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr goal_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_ = State::WAITING_START;
    std::vector<double> current_pos_ = {0.0, 0.0, 0.0};
    std::vector<double> target_pos_ = {0.0, 0.0, 0.0};
    bool pos_received_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<TrajectoryManager>();
    
    // !!! IMPORTANTE !!!
    // Usiamo MultiThreadedExecutor perché il servizio rimarrà bloccato nel ciclo while
    // aspettando che il drone finisca. Se usassimo SingleThreaded, il timer si bloccherebbe
    // e il drone non si muoverebbe mai.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}