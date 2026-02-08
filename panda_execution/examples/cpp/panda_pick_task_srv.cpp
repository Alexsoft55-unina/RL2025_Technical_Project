#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>
#include <mutex>

// HEADER DEL TUO SERVIZIO CUSTOM
#include "panda_execution/srv/move_to_pose.hpp" 

// MoveIt Task Constructor includes
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h> // <--- HEADER NECESSARIO
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/robot_model/robot_model.h>       
#include <moveit/task_constructor/stages/move_relative.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace moveit::task_constructor;
using std::placeholders::_1;
using std::placeholders::_2;

class PandaSimpleMTCNode : public rclcpp::Node {
public:
    using ArmAction = control_msgs::action::FollowJointTrajectory;
    using MoveToPoseSrv = panda_execution::srv::MoveToPose; 

    PandaSimpleMTCNode(const rclcpp::NodeOptions& options) : Node("panda_pick_task_srv", options) {
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        service_server_ = this->create_service<MoveToPoseSrv>(
            "start_panda_pick", 
            std::bind(&PandaSimpleMTCNode::handle_service_request, this, _1, _2),
            rmw_qos_profile_services_default,
            callback_group_);

        arm_action_client_ = rclcpp_action::create_client<ArmAction>(
            this, 
            "/joint_trajectory_controller/follow_joint_trajectory",
            callback_group_);

        RCLCPP_INFO(this->get_logger(), "NODO PRONTO. Servizio: /start_panda_pick (Richiede Posa + cartesian_mode)");
    }

    // --- VERSIONE MODIFICATA: SOLO CHECK, NESSUN MOVIMENTO ---
    bool ping_arm_controller() {
        // Controlla solo se il server dell'azione è disponibile
        if (!arm_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Controller Braccio NON TROVATO! Impossibile muovere.");
            return false;
        }
        // Ritorna true subito, senza inviare traiettorie di reset
        return true; 
    }

    // --- MODIFICA: Aggiunto parametro bool use_cartesian ---
    Task create_move_task(double x, double y, double z, double r, double p, double yaw, bool use_cartesian, double lift_height = 0.3) {
        Task t;
        t.stages()->setName(use_cartesian ? "Cartesian Move Task" : "Pipeline Move Task");
        t.loadRobotModel(shared_from_this());

        // 1. Current State
        t.add(std::make_unique<stages::CurrentState>("current-state"));

        // 2. Ignora Collisioni
        {
            auto allow_collision = std::make_unique<stages::ModifyPlanningScene>("allow-collision");
            allow_collision->allowCollisions("panda_hand", true);
            allow_collision->allowCollisions("panda_leftfinger", true);
            allow_collision->allowCollisions("panda_rightfinger", true);
            allow_collision->allowCollisions("panda_link8", true); // Spesso il link8 tocca il link7 o la mano
            allow_collision->allowCollisions("panda_link7", true); // Polso
            allow_collision->allowCollisions("panda_link6", true); 
            allow_collision->allowCollisions("panda_link5", true); 
            allow_collision->allowCollisions("panda_link4", true); 
            allow_collision->allowCollisions("panda_link3", true); 
            allow_collision->allowCollisions("panda_link2", true); 
            allow_collision->allowCollisions("panda_link1", true); 
            allow_collision->allowCollisions("panda_link0", true); 
            t.add(std::move(allow_collision));
        }
        // --- NUOVO STAGE: LIFT INIZIALE ---
        {
            // Usiamo un solver Cartesiano specifico per il lift per garantire una linea retta verticale
            auto cartesian_solver = std::make_shared<solvers::CartesianPath>();
            cartesian_solver->setMaxVelocityScalingFactor(1.0);
            cartesian_solver->setStepSize(0.01);

            auto lift = std::make_unique<stages::MoveRelative>("initial_lift", cartesian_solver);
            lift->setGroup("arm");
            lift->setIKFrame("panda_hand_tcp"); // Muove il TCP
            
            // Impostiamo la direzione: Z positivo rispetto al WORLD
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "panda_link0"; 
            vec.vector.x = -0.4;
            vec.vector.y = 0.0;
            vec.vector.z = lift_height; // Esempio: 0.1 metri verso l'alto
            
            lift->setDirection(vec);
            t.add(std::move(lift));
        }
        // --- 3. SELEZIONE DEL PLANNER (Connect Stage) ---
        stages::Connect::GroupPlannerVector planners;
        
        if (use_cartesian) {
            // A. Cartesian Planner (Linea Retta)
            auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
            cartesian_planner->setMaxVelocityScalingFactor(0.1);
            cartesian_planner->setStepSize(0.01); // Passo di campionamento (1cm)
            cartesian_planner->setJumpThreshold(0.0); // Ignora salti nei giunti (risoluzione ibrida)
            planners.push_back({"arm", cartesian_planner});
            RCLCPP_INFO(this->get_logger(), "Usando CARTESIAN PLANNER (Linear)");

        } else {
            // B. Pipeline Planner (OMPL / RRTConnect - Evita ostacoli)
            auto pipeline_planner = std::make_shared<solvers::PipelinePlanner>(shared_from_this(), "ompl");
            pipeline_planner->setPlannerId("RRTConnectkConfigDefault");
            pipeline_planner->setMaxVelocityScalingFactor(1.0); 
            pipeline_planner->setProperty("planning_time", 5.0);
            planners.push_back({"arm", pipeline_planner});
            RCLCPP_INFO(this->get_logger(), "Usando PIPELINE PLANNER (OMPL)");

        }

        auto connect = std::make_unique<stages::Connect>("connect-to-target", planners);
        connect->setTimeout(5.0);
        t.add(std::move(connect));
        // ------------------------------------------------

        auto generate_pose = std::make_unique<stages::GeneratePose>("generate-pose");
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "world";

        target_pose.pose.position.x = x; 
        target_pose.pose.position.y = y; 
        target_pose.pose.position.z = z; 
        tf2::Quaternion q;
        q.setRPY(r, p, yaw); 
        target_pose.pose.orientation = tf2::toMsg(q);

        generate_pose->setPose(target_pose);
        generate_pose->setMonitoredStage(t.stages()->findChild("current-state"));

        auto compute_ik = std::make_unique<stages::ComputeIK>("compute-ik", std::move(generate_pose));
        compute_ik->setMaxIKSolutions(20);
        compute_ik->setGroup("arm");
        compute_ik->setIKFrame("panda_hand_tcp"); 
        compute_ik->setIgnoreCollisions(true); 
        compute_ik->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});
        
        t.add(std::move(compute_ik));

        // --- NUOVO STAGE: Place ---
        {
            // Usiamo un solver Cartesiano specifico per il lift per garantire una linea retta verticale
            auto cartesian_solver = std::make_shared<solvers::CartesianPath>();
            cartesian_solver->setMaxVelocityScalingFactor(1.0);
            cartesian_solver->setStepSize(0.01);

            auto place = std::make_unique<stages::MoveRelative>("placing", cartesian_solver);
            place->setGroup("arm");
            place->setIKFrame("panda_hand_tcp"); // Muove il TCP
            
            // Impostiamo la direzione: Z positivo rispetto al WORLD
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "panda_link0"; 
            vec.vector.x = -0.0;
            vec.vector.y = 0.0;
            vec.vector.z = -0.0; // -0.1 
            
            place->setDirection(vec);
            t.add(std::move(place));
        }
         // --- NUOVO STAGE: aggiustamento ---
        {
            // Usiamo un solver Cartesiano specifico per il lift per garantire una linea retta verticale
            auto cartesian_solver = std::make_shared<solvers::CartesianPath>();
            cartesian_solver->setMaxVelocityScalingFactor(0.5);
            cartesian_solver->setStepSize(0.01);

            auto adjustment = std::make_unique<stages::MoveRelative>("adjustment", cartesian_solver);
            adjustment->setGroup("arm");
            adjustment->setIKFrame("panda_hand_tcp"); // Muove il TCP
            
            // Impostiamo la direzione: Z positivo rispetto al WORLD
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world"; 
            vec.vector.x = -0.15;
            vec.vector.y = 0.0;
            vec.vector.z = 0.0; // Esempio: 0.1 metri verso l'alto
            
            adjustment->setDirection(vec);
            t.add(std::move(adjustment));
        }

        return t;
    }

void handle_service_request(
        const std::shared_ptr<MoveToPoseSrv::Request> request,
        std::shared_ptr<MoveToPoseSrv::Response> response) 
    {
        RCLCPP_INFO(this->get_logger(), "Target: [%.2f, %.2f, %.2f] | Mode: %s",
            request->x, request->y, request->z, request->cartesian_mode ? "CARTESIAN" : "PIPELINE");

        // 1. Ping Check
        if (!this->ping_arm_controller()) {
            RCLCPP_ERROR(this->get_logger(), "Reset fallito.");
            response->success = false;
            response->message = "Controller non disponibile";
            return;
        }

        // 2. Creazione Task
        move_task_ = create_move_task(
            request->x, request->y, request->z, 
            request->roll, request->pitch, request->yaw, 
            request->cartesian_mode
        );

        // 3. Esecuzione Sincrona (Bloccante)
        try {
            move_task_.init();
            if (move_task_.plan(5)) {
                RCLCPP_INFO(this->get_logger(), "Piano trovato! Esecuzione...");
                
                // execute() blocca finché l'azione non è finita
                auto result = move_task_.execute(*move_task_.solutions().front());
                
                if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                     RCLCPP_INFO(this->get_logger(), "Target Raggiunto! 🎯");
                     response->success = true;
                     response->message = "Target Raggiunto";
                } else {
                     response->success = false;
                     response->message = "Errore durante l'esecuzione fisica";
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Pianificazione fallita.");
                response->success = false;
                response->message = "Pianificazione fallita";
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Eccezione: %s", e.what());
            response->success = false;
            response->message = e.what();
        }
    }

private:
    rclcpp::Service<MoveToPoseSrv>::SharedPtr service_server_; 
    rclcpp_action::Client<ArmAction>::SharedPtr arm_action_client_;
    rclcpp::CallbackGroup::SharedPtr callback_group_; 
    moveit::task_constructor::Task move_task_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PandaSimpleMTCNode>(rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}