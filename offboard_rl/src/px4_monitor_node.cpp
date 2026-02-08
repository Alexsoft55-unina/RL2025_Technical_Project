#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"

// Header dei servizi custom
#include "offboard_rl/srv/drone_pose.hpp"
#include "offboard_rl/srv/check_at_base.hpp" // NUOVO HEADER

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class Px4MonitorNode : public rclcpp::Node
{
public:
    Px4MonitorNode() : Node("px4_monitor_node")
    {
        // QoS per PX4
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // Subscriber Odometria
        subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            qos,
            std::bind(&Px4MonitorNode::odometry_callback, this, _1));

        // Service 1: Check Base (ORA USA CheckAtBase.srv)
        srv_check_base_ = this->create_service<offboard_rl::srv::CheckAtBase>(
            "check_if_at_base",
            std::bind(&Px4MonitorNode::check_base_callback, this, _1, _2));

        // Service 2: Get Pose
        srv_get_pose_ = this->create_service<offboard_rl::srv::DronePose>(
            "get_drone_pose_enu",
            std::bind(&Px4MonitorNode::get_pose_callback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Nodo PX4 Monitor (Logica Panda Base) avviato.");
    }

private:
    bool has_data_ = false;
    geometry_msgs::msg::Pose current_pose_enu_;
    
    // Tolleranze
    const float DISTANCE_TOLERANCE = 1.5f; // Distanza massima dal Panda (metri)
    const float LANDING_TOLERANCE = 0.15f; // Altezza massima per considerarlo "atterrato" (metri)

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
    
    // Aggiornato al nuovo tipo di servizio
    rclcpp::Service<offboard_rl::srv::CheckAtBase>::SharedPtr srv_check_base_;
    rclcpp::Service<offboard_rl::srv::DronePose>::SharedPtr srv_get_pose_;

    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        has_data_ = true;

        // Conversione NED -> ENU
        current_pose_enu_.position.x = msg->position[1];
        current_pose_enu_.position.y = msg->position[0];
        current_pose_enu_.position.z = -msg->position[2];

        current_pose_enu_.orientation.w = msg->q[0];
        current_pose_enu_.orientation.x = msg->q[2]; 
        current_pose_enu_.orientation.y = msg->q[1];
        current_pose_enu_.orientation.z = -msg->q[3];
    }

    // Callback aggiornata con i dati del Panda in ingresso
    void check_base_callback(
        const std::shared_ptr<offboard_rl::srv::CheckAtBase::Request> request,
        std::shared_ptr<offboard_rl::srv::CheckAtBase::Response> response)
    {
        if (!has_data_) {
            response->is_at_base = false;
            response->message = "Attesa dati odometria drone...";
            response->distance_from_panda = -1.0;
            return;
        }

        // 1. Calcolo Distanza Euclidea (Drone vs Panda)
        // Usiamo le coordinate ricevute nella Request (panda_x, panda_y, panda_z)
        float dx = current_pose_enu_.position.x - request->panda_x;
        float dy = current_pose_enu_.position.y - request->panda_y;
        float dz = current_pose_enu_.position.z - request->panda_z; // Consideriamo anche Z per la distanza totale

        float distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2) + std::pow(dz, 2));
        response->distance_from_panda = distance;

        // 2. Controllo condizione "Atterrato"
        // In ENU, Z=0 è il suolo. Se Z < tolleranza (es. 15cm), è atterrato.
        bool is_landed = std::abs(current_pose_enu_.position.z) < LANDING_TOLERANCE;

        // 3. Controllo condizione "Vicino al Panda"
        bool is_close_enough = distance < DISTANCE_TOLERANCE;

        // Logica finale: DEVE essere atterrato E vicino al Panda
        if (is_landed && is_close_enough) {
            response->is_at_base = true;
            response->message = "IN BASE (Atterrato e vicino al Panda)";
        } else {
            response->is_at_base = false;
            if (!is_landed && is_close_enough) {
                response->message = "FUORI BASE: Vicino al Panda ma in volo";
            } else if (is_landed && !is_close_enough) {
                response->message = "FUORI BASE: Atterrato ma lontano dal Panda";
            } else {
                response->message = "FUORI BASE: In volo e lontano";
            }
        }
    }

    void get_pose_callback(
        const std::shared_ptr<offboard_rl::srv::DronePose::Request>,
        std::shared_ptr<offboard_rl::srv::DronePose::Response> response)
    {
        if (!has_data_) {
            response->success = false;
            return;
        }
        response->success = true;
        response->pose = current_pose_enu_;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4MonitorNode>());
    rclcpp::shutdown();
    return 0;
}