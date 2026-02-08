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

    PandaSimpleMTCNode(const rclcpp::NodeOptions& options) : Node("panda_move_task_node", options) {
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        service_server_ = this->create_service<MoveToPoseSrv>(
            "panda_move_task", 
            std::bind(&PandaSimpleMTCNode::handle_service_request, this, _1, _2),
            rmw_qos_profile_services_default,
            callback_group_);

        arm_action_client_ = rclcpp_action::create_client<ArmAction>(
            this, 
            "/joint_trajectory_controller/follow_joint_trajectory",
            callback_group_);

        RCLCPP_INFO(this->get_logger(), "NODO PRONTO. Servizio: /panda_move_task (Richiede Posa + cartesian_mode)");
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
    Task create_move_task(double x, double y, double z, double r, double p, double yaw, bool use_cartesian) {
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

        // --- 3. SELEZIONE DEL PLANNER (Connect Stage) ---
        stages::Connect::GroupPlannerVector planners;

        // ---------------RELEASE---------------------------------
        
            auto pipeline_planner = std::make_shared<solvers::PipelinePlanner>(shared_from_this(), "ompl");
            pipeline_planner->setPlannerId("RRTConnectkConfigDefault");
            pipeline_planner->setMaxVelocityScalingFactor(1.0); 
            pipeline_planner->setProperty("planning_time", 5.0);
            planners.push_back({"arm", pipeline_planner});

            auto cartesian_solver = std::make_shared<solvers::CartesianPath>();
            cartesian_solver->setMaxVelocityScalingFactor(1.0);
            cartesian_solver->setStepSize(0.01);

            auto release = std::make_unique<stages::MoveRelative>("release", cartesian_solver);
            release->setGroup("arm");
            release->setIKFrame("panda_hand_tcp"); // Muove il TCP
            
            // Impostiamo la direzione: Z positivo rispetto al WORLD
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "panda_link0"; 
            vec.vector.x = 0.3;
            vec.vector.y = 0.0;
            vec.vector.z = 0.0; // Esempio: 0.1 metri verso l'alto
            
            release->setDirection(vec);
            release->setMinMaxDistance(0.1, 0.3);
            t.add(std::move(release));

            auto up = std::make_unique<stages::MoveRelative>("move_on_up", pipeline_planner);
            up->setGroup("arm");
            up->setIKFrame("panda_hand_tcp"); // Muove il TCP

            vec.header.frame_id = "panda_link0"; 
            vec.vector.x = 0.0;
            vec.vector.y = 0.0;
            vec.vector.z = 0.5; // Esempio: 0.1 metri verso l'alto
            
            up->setDirection(vec);
            t.add(std::move(up));

        auto connect = std::make_unique<stages::Connect>("connect-to-target", planners);
        connect->setTimeout(5.0);
        connect->properties().configureInitFrom(Stage::PARENT);
        t.add(std::move(connect));

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
        return t;
    }
void handle_service_request(
        const std::shared_ptr<MoveToPoseSrv::Request> request,
        std::shared_ptr<MoveToPoseSrv::Response> response) 
    {
        RCLCPP_INFO(this->get_logger(), ">>> Richiesta MoveTo: [%.2f, %.2f, %.2f]", request->x, request->y, request->z);

        if (!this->ping_arm_controller()) {
            response->success = false;
            response->message = "Controller non disponibile";
            return;
        }

        // --- ESECUZIONE SINCRONA (Senza Thread Detach) ---
        // Questo blocca la risposta del servizio finché il robot non ha finito.
        // Grazie a MultiThreadedExecutor, gli altri callback continuano a funzionare.
        
        try {
            move_task_.reset(); // Pulisce il task precedente
            move_task_ = create_move_task(request->x, request->y, request->z, 
                                          request->roll, request->pitch, request->yaw, 
                                          request->cartesian_mode);

            move_task_.init();

            if (move_task_.plan(5)) {
                RCLCPP_INFO(this->get_logger(), "Piano trovato. Esecuzione...");
                
                auto result = move_task_.execute(*move_task_.solutions().front());
                
                if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                    RCLCPP_INFO(this->get_logger(), "Target Raggiunto! 🎯");
                    response->success = true;
                    response->message = "Target Raggiunto";
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Errore Esecuzione: %d", result.val);
                    response->success = false;
                    response->message = "Errore durante esecuzione traiettoria";
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Pianificazione fallita (No Solution).");
                response->success = false;
                response->message = "Pianificazione fallita";
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Eccezione: %s", e.what());
            response->success = false;
            response->message = e.what();
        }
        
        // Quando la funzione termina qui, il Client riceve la risposta.
        // Poiché abbiamo aspettato la fine dell'execute, il Client è ora sincronizzato col robot.
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
    
    // IMPORTANTE: MultiThreadedExecutor è necessario per la modalità sincrona
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}