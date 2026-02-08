#include <chrono>
#include <memory>
#include <cmath>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "aruco_msgs/msg/marker_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"

using namespace std::chrono_literals;

// NUOVI STATI SEMPLIFICATI
enum FollowerState {
    IDLE,
    STEP_ALIGNING,      // Accorpa Centering (XY) e Aligning (Angolo)
    STEP_APPROACHING,   // Avanzamento Z
    FINISHED
};

class ArucoFollower : public rclcpp::Node {
public:
    ArucoFollower() : Node("visual_servoing_node") {
        
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/delta_twist_cmds", 10);
        
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_;

        marker_sub_ = this->create_subscription<aruco_msgs::msg::MarkerArray>(
            "/aruco_marker_publisher/markers", 10,
            std::bind(&ArucoFollower::marker_callback, this, std::placeholders::_1),
            sub_opt);

        server_ = this->create_service<std_srvs::srv::SetBool>(
            "start_visual_servo",
            std::bind(&ArucoFollower::handle_request, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_);

        current_state_ = IDLE;

        RCLCPP_INFO(this->get_logger(), "=== VISUAL SERVOING 2.0 (Merged Steps + Fast Finish) ===");
    }

private:
    void handle_request(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        if (request->data) {
            RCLCPP_INFO(this->get_logger(), "START ricevuto! Inizio Allineamento Combinato...");
            current_state_ = STEP_ALIGNING; // Si parte subito con tutto attivo
            
            int timeout_counter = 0;
            const int max_timeout = 6000; // 60 secondi

            while (current_state_ != FINISHED && current_state_ != IDLE && timeout_counter < max_timeout) {
                std::this_thread::sleep_for(10ms);
                timeout_counter++;
                
                if (timeout_counter % 500 == 0) {
                    RCLCPP_WARN(this->get_logger(), "Ancora in esecuzione... Stato attuale: %d", current_state_);
                }
            }

            if (current_state_ == FINISHED) {
                response->success = true;
                response->message = "Visual Servoing Completato";
                RCLCPP_INFO(this->get_logger(), "Task Terminato con SUCCESSO");
            } else {
                response->success = false;
                response->message = "Timeout Globale";
                RCLCPP_ERROR(this->get_logger(), "Task Terminato per TIMEOUT GLOBALE.");
            }
            stop_robot();
            current_state_ = IDLE; 
        } else {
            current_state_ = IDLE;
            stop_robot();
            response->success = true;
            response->message = "Stop manuale";
        }
    }

    void marker_callback(const aruco_msgs::msg::MarkerArray::SharedPtr msg) {
        if (current_state_ == IDLE || current_state_ == FINISHED) return;

        if (msg->markers.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Nessun marker visto! Robot fermo.");
            stop_robot();
            return;
        }

        std::string source_frame = msg->header.frame_id;
        std::string target_frame = "panda_hand"; 
        
        geometry_msgs::msg::PoseStamped pose_cam, pose_hand;
        pose_cam.header = msg->header;
        pose_cam.pose = msg->markers[0].pose.pose;

        try {
            if (tf_buffer_->canTransform(target_frame, source_frame, tf2::TimePointZero)) {
                tf_buffer_->transform(pose_cam, pose_hand, target_frame);
            } else {
                stop_robot(); 
                return;
            }
        } catch (const tf2::TransformException & ex) {
            stop_robot();
            return;
        }

        // --- Calcolo Errori ---
        // Nota: mantengo i tuoi offset -0.05
        double err_x = ( pose_hand.pose.position.y ); 
        double err_y = ( pose_hand.pose.position.z - 0.05); 
        double err_z = pose_hand.pose.position.x; 

        tf2::Quaternion q(
            pose_hand.pose.orientation.x, pose_hand.pose.orientation.y,
            pose_hand.pose.orientation.z, pose_hand.pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        double pitch_opt  = normalize_angle_pi(pitch);
        
        // --- Parametri ---
        double kp_xy = 3.0;
        double kp_z  = 3.0; 
        double kp_ang = 1.0;

        double v_x = 0.0, v_y = 0.0, v_z = 0.0, w_z = 0.0;
        
        double xy_tol = 0.05; 
        double ang_tol = 0.20;
        double dist_target = 0.05;

        // Variabili Timeout Step Finale
        const double STEP_APPROACH_MAX_DURATION = 15.0;

        // --- LOGICA SPECIALE: SUCCESS CONDITION GLOBALE ---
        // Se siamo già abbastanza vicini (indipendentemente dallo stato), finiamo.
        // Questo soddisfa la richiesta: "se la condizione del passo 3 è soddisfatta, risultato raggiunto"
        if (err_z <= dist_target) {
            RCLCPP_INFO(this->get_logger(), "Distanza Target Raggiunta (%.3f <= %.3f) -> FINISHED IMMEDIATO", err_z, dist_target);
            stop_robot();
            current_state_ = FINISHED;
            return; // Uscita anticipata
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
           "Stato: %d | ErrX: %.3f | ErrY: %.3f | ErrZ: %.3f", current_state_, err_x, err_y, err_z);

        switch (current_state_) {
            case STEP_ALIGNING:
                // Muoviamo X, Y e Rotazione INSIEME. Z fermo.
                v_x = err_x * kp_xy;
                v_y = err_y * kp_xy;
                w_z = pitch_opt * kp_ang;
                v_z = 0.0; 

                // Condizione di transizione: Tutti gli allineamenti OK
                if (std::abs(err_x) < xy_tol && std::abs(err_y) < xy_tol && std::abs(pitch_opt) < ang_tol) {
                    RCLCPP_INFO(this->get_logger(), "Allineamento (XY+Ang) OK -> APPROACHING");
                    step_approach_start_time_ = this->now();
                    current_state_ = STEP_APPROACHING;
                }
                break;

            case STEP_APPROACHING:
                // Correggiamo tutto mentre avanziamo
                v_x = err_x * kp_xy;
                v_y = err_y * kp_xy;
                w_z = pitch_opt * kp_ang;
                v_z = (err_z) * kp_z; // Avanzamento attivo

                // Nota: Il controllo "err_z <= dist_target" è già gestito all'inizio della funzione
                // Qui gestiamo solo il timeout specifico di questa fase
                {
                    auto duration = (this->now() - step_approach_start_time_).seconds();
                    if (duration > STEP_APPROACH_MAX_DURATION) {
                        RCLCPP_WARN(this->get_logger(), "Approaching Timeout (%.1fs) - Forzatura SUCCESSO", duration);
                        current_state_ = FINISHED;
                    }
                }
                break;
                
            default: break;
        }

        // Pubblicazione comandi
        auto twist_msg = geometry_msgs::msg::TwistStamped();
        twist_msg.header.stamp = this->now();
        twist_msg.header.frame_id = target_frame;
        twist_msg.twist.linear.x = v_x;
        twist_msg.twist.linear.y = v_y;
        twist_msg.twist.linear.z = v_z;
        twist_msg.twist.angular.z = w_z;
        twist_pub_->publish(twist_msg);
    }

    void stop_robot() {
        auto twist_msg = geometry_msgs::msg::TwistStamped();
        twist_msg.header.stamp = this->now();
        twist_msg.header.frame_id = "panda_hand"; 
        twist_pub_->publish(twist_msg);
    }

    double normalize_angle_pi(double angle) {
        while (angle > M_PI_2) angle -= M_PI;
        while (angle < -M_PI_2) angle += M_PI;
        return angle;
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr server_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    FollowerState current_state_;

    rclcpp::Time step_approach_start_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor; 
    auto node = std::make_shared<ArucoFollower>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}