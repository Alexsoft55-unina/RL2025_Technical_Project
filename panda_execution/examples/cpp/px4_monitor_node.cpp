#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose.hpp"

// Includiamo l'header generato dal nostro nuovo servizio custom
// NOTA: Sostituisci 'tuo_pacchetto' con il nome reale del pacchetto
#include "tuo_pacchetto/srv/get_drone_pose.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class Px4MonitorNode : public rclcpp::Node
{
public:
    Px4MonitorNode() : Node("px4_monitor_node")
    {
        // QoS per PX4 (Best Effort è obbligatorio)
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // Subscriber Odometria PX4
        subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            qos,
            std::bind(&Px4MonitorNode::odometry_callback, this, _1));

        // Service 1: Check Base (Trigger standard)
        srv_check_base_ = this->create_service<std_srvs::srv::Trigger>(
            "check_if_at_base",
            std::bind(&Px4MonitorNode::check_base_callback, this, _1, _2));

        // Service 2: Get Pose (Custom Service con geometry_msgs/Pose)
        srv_get_pose_ = this->create_service<tuo_pacchetto::srv::GetDronePose>(
            "get_drone_pose_enu",
            std::bind(&Px4MonitorNode::get_pose_callback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Nodo PX4 Monitor (NED->ENU) avviato.");
    }

private:
    bool has_data_ = false;
    
    // Salviamo direttamente la posa in formato ROS (ENU)
    geometry_msgs::msg::Pose current_pose_enu_;

    const float BASE_TOLERANCE = 0.5f;

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_check_base_;
    rclcpp::Service<tuo_pacchetto::srv::GetDronePose>::SharedPtr srv_get_pose_;

    // --- CONVERSIONE NED -> ENU ---
    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        has_data_ = true;

        // PX4 usa NED (North-East-Down). ROS usa ENU (East-North-Up).
        // Conversione Posizione:
        // X (ROS/East)  = Y (PX4/East)
        // Y (ROS/North) = X (PX4/North)
        // Z (ROS/Up)    = -Z (PX4/Down)
        
        current_pose_enu_.position.x = msg->position[1];
        current_pose_enu_.position.y = msg->position[0];
        current_pose_enu_.position.z = -msg->position[2];

        // Conversione Orientamento (Quaternione):
        // La rotazione da NED a ENU comporta una rotazione del frame.
        // Mappatura semplificata dei quaternioni (x->y, y->x, z->-z, w->w)
        // Nota: Per conversioni precise di assetto complesso, si consiglia tf2::Quaternion.
        
        current_pose_enu_.orientation.w = msg->q[0];
        current_pose_enu_.orientation.x = msg->q[2]; 
        current_pose_enu_.orientation.y = msg->q[1];
        current_pose_enu_.orientation.z = -msg->q[3];
    }

    // Server 1: Verifica se in base
    void check_base_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!has_data_) {
            response->success = false;
            response->message = "Attesa dati odometria...";
            return;
        }

        // Calcolo distanza su ENU (matematicamente identico a NED per la norma)
        float distance = std::sqrt(
            std::pow(current_pose_enu_.position.x, 2) + 
            std::pow(current_pose_enu_.position.y, 2) + 
            std::pow(current_pose_enu_.position.z, 2)
        );

        if (distance < BASE_TOLERANCE) {
            response->success = true;
            response->message = "IN BASE";
        } else {
            response->success = false;
            response->message = "FUORI BASE";
        }
    }

    // Server 2: Restituisce la posa ENU
    void get_pose_callback(
        const std::shared_ptr<tuo_pacchetto::srv::GetDronePose::Request>,
        std::shared_ptr<tuo_pacchetto::srv::GetDronePose::Response> response)
    {
        if (!has_data_) {
            response->success = false;
            return;
        }

        response->success = true;
        response->pose = current_pose_enu_; // Posa già convertita in callback
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4MonitorNode>());
    rclcpp::shutdown();
    return 0;
}