#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <offboard_rl/utils.h>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using Eigen::Vector4d;

class GoToPoint : public rclcpp::Node
{
public:
    GoToPoint() : Node("go_to_point")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // Subscriptions
        local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos, std::bind(&GoToPoint::vehicle_local_position_callback, this, std::placeholders::_1));
        
        attitude_subscription_ = this->create_subscription<VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos, std::bind(&GoToPoint::vehicle_attitude_callback, this, std::placeholders::_1));

        // Subscriber per ricevere i goal
        goal_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/go_to_point/goal", 10, std::bind(&GoToPoint::goal_callback, this, std::placeholders::_1));

        // Publishers
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        // Timers
        timer_offboard_ = this->create_wall_timer(100ms, std::bind(&GoToPoint::activate_offboard, this));
        timer_trajectory_publish_ = this->create_wall_timer(20ms, std::bind(&GoToPoint::publish_trajectory_setpoint, this));

        RCLCPP_INFO(this->get_logger(), "Server pronto. In ascolto su /go_to_point/goal");
    }

private:
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscription_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr attitude_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr goal_subscription_;
    
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

    rclcpp::TimerBase::SharedPtr timer_offboard_;
    rclcpp::TimerBase::SharedPtr timer_trajectory_publish_;

    // --- State Variables ---
    bool set_point_received{false};
    bool offboard_active{false};
    bool trajectory_computed{false};
    bool landing_mode{false}; // <--- NUOVO FLAG
    
    Eigen::Vector<double, 6> x; 
    double T{0.0}, t{0.0};
    Vector4d pos_i, pos_f;
    
    VehicleLocalPosition current_position_{};
    VehicleAttitude current_attitude_{};
    int offboard_counter{0};

    // --- Callbacks ---

    void goal_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 5) {
            RCLCPP_WARN(this->get_logger(), "Input errato! Inviare [x, y, z, yaw, T]");
            return;
        }

        // --- GESTIONE ATTERRAGGIO (MODIFICA PRINCIPALE) ---
        if (msg->data[4] == -1.0) {
            RCLCPP_INFO(this->get_logger(), "!!! Ricevuto comando LAND (T=-1) !!!");
            
            // 1. Attiva la modalità landing
            landing_mode = true;
            
            // 2. Disabilita l'invio della traiettoria Offboard (fondamentale!)
            set_point_received = false; 

            // 3. Invia il comando NAV_LAND a PX4
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
            return;
        }

        // --- GESTIONE TRAIETTORIA NORMALE ---
        landing_mode = false; // Reset nel caso arrivi un nuovo punto dopo il landing

        // Se stavamo già volando, pos_i diventa la posizione attuale
        pos_i(0) = current_position_.x;
        pos_i(1) = current_position_.y;
        pos_i(2) = current_position_.z;
        auto rpy = utilities::quatToRpy(Vector4d(current_attitude_.q[0], current_attitude_.q[1], current_attitude_.q[2], current_attitude_.q[3]));
        pos_i(3) = rpy[2];

        // Nuovo target
        pos_f(0) = msg->data[0];
        pos_f(1) = msg->data[1];
        pos_f(2) = -msg->data[2]; // Conversione in NED (Z giù)
        pos_f(3) = msg->data[3];
        T = msg->data[4];

        // Reset per nuova traiettoria
        t = 0.0;
        trajectory_computed = false;
        set_point_received = true;

        RCLCPP_INFO(this->get_logger(), "Nuovo Goal: x=%.2f y=%.2f z=%.2f | T=%.2f", pos_f(0), pos_f(1), pos_f(2), T);
    }

    void vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg) { current_position_ = *msg; }
    void vehicle_attitude_callback(const VehicleAttitude::SharedPtr msg) { current_attitude_ = *msg; }

    void activate_offboard()
    {
        // Se siamo in landing mode, NON vogliamo forzare l'Offboard mode
        if (set_point_received && !landing_mode)
        {
            OffboardControlMode msg_mode{};
            msg_mode.position = true;
            msg_mode.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            offboard_control_mode_publisher_->publish(msg_mode);

            if (offboard_counter == 10) {
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // Mode Offboard
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); // Arm
                offboard_active = true;
            }

            if (offboard_counter <= 10) offboard_counter++;
        }
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }

    void publish_trajectory_setpoint()
    {
        // Se siamo in landing mode, NON pubblichiamo setpoint di posizione
        if (!set_point_received || !offboard_active || t > T || landing_mode) return;

        double dt = 0.02; // 20ms
        TrajectorySetpoint msg = compute_trajectory_setpoint(t);
        trajectory_setpoint_publisher_->publish(msg);
        t += dt;
    }

    TrajectorySetpoint compute_trajectory_setpoint(double t_val)
    {
        Vector4d e = pos_f - pos_i;
        e(3) = utilities::angleError(pos_f(3), pos_i(3));
        double s_f = e.norm();

        if (!trajectory_computed)
        {
            Eigen::Matrix<double, 6, 6> A;
            Eigen::VectorXd b(6);
            b << 0.0, 0.0, 0.0, s_f, 0.0, 0.0;
            
            A << 0, 0, 0, 0, 0, 1,
                 0, 0, 0, 0, 1, 0,
                 0, 0, 0, 2, 0, 0, 
                 pow(T,5), pow(T,4), pow(T,3), pow(T,2), T, 1,
                 5*pow(T,4), 4*pow(T,3), 3*pow(T,2), 2*T, 1, 0,
                 20*pow(T,3), 12*pow(T,2), 6*T, 2, 0, 0;

            x = A.inverse() * b;
            trajectory_computed = true;
        }

        double s = x(0)*pow(t_val,5) + x(1)*pow(t_val,4) + x(2)*pow(t_val,3) + x(3)*pow(t_val,2) + x(4)*t_val + x(5);
        double sd = 5*x(0)*pow(t_val,4) + 4*x(1)*pow(t_val,3) + 3*x(2)*pow(t_val,2) + 2*x(3)*t_val + x(4);
        double sdd = 20*x(0)*pow(t_val,3) + 12*x(1)*pow(t_val,2) + 6*x(2)*t_val + 2*x(3);

        TrajectorySetpoint msg{};
        msg.position = {float(pos_i(0) + s*e(0)/s_f), float(pos_i(1) + s*e(1)/s_f), float(pos_i(2) + s*e(2)/s_f)};
        msg.velocity = {float(sd*e(0)/s_f), float(sd*e(1)/s_f), float(sd*e(2)/s_f)};
        msg.acceleration = {float(sdd*e(0)/s_f), float(sdd*e(1)/s_f), float(sdd*e(2)/s_f)};
        msg.yaw = float(pos_i(3) + s*e(3)/s_f);
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        return msg;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoToPoint>());
    rclcpp::shutdown();
    return 0;
}