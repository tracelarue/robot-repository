#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
    #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
    #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
    #include <tf2_eigen/tf2_eigen.hpp>
#else
    #include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode {
public:
    MTCTaskNode(const rclcpp::NodeOptions& options);

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

    void doTask();

    void setupPlanningScene();

private:
    // Compose an MTC task from a series of stages.
    mtc::Task createTask();
    mtc::Task task_;
    rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface() {
    return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) } {}

void MTCTaskNode::setupPlanningScene() {
    moveit_msgs::msg::CollisionObject object;
    object.id = "object";
    object.header.frame_id = "world";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = { 0.05, 0.02 };

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.5;
    pose.position.y = 0;
    // to put the object above surface we bring the half that is below ground to top.
    pose.position.z = 0.5 * 0.05;
    object.pose = pose;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask() {
    task_ = createTask();

    try
    {
      task_.init();
    }
    catch (mtc::InitStageException& e)
    {
      RCLCPP_ERROR_STREAM(LOGGER, e);
      return;
    }

    if (!task_.plan(5))
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
      return;
    }
    task_.introspection().publishSolution(*task_.solutions().front());

    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
      return;
    }

    return;
}

mtc::Task MTCTaskNode::createTask() {
    mtc::Task task;
    task.stages()->setName("chessaton demo task");
    task.loadRobotModel(node_);

    const auto& arm_group_name = "arm_group";
    const auto& hand_group_name = "hand_group";
    const auto& hand_frame = "end_effector_frame";

    // Set task properties
    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", hand_frame);

    // Sampling planner
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    // Cartesian planner
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.001);

    // Stages

	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ****************************************************/
    mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator

    {
        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
        current_state_ptr = stage_state_current.get();
        task.add(std::move(stage_state_current));
    }

    /****************************************************
	 *                                                  *
	 *               Open Hand                          *
	 *                                                  *
	 ****************************************************/
    {
        auto stage_open_hand =
            std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
        stage_open_hand->setGroup(hand_group_name);
        stage_open_hand->setGoal("hand_open");
        task.add(std::move(stage_open_hand));
    }

    /****************************************************
	 *                                                  *
	 *               Move to Pick                       *
	 *                                                  *
	 ****************************************************/
    {
        auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
        "move to pick",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
        stage_move_to_pick->setTimeout(5.0);
        stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage_move_to_pick));
    }

    mtc::Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator

    /****************************************************
	 *                                                  *
	 *               Pick Object                        *
	 *                                                  *
     ****************************************************/
    {
            auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
            task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
            grasp->properties().configureInitFrom(mtc::Stage::PARENT,{ "eef", "group", "ik_frame" });

        /****************************************************
    	 *                Approach Object                   *
         ****************************************************/
        {
            auto stage =
            std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.01, 0.03);

            // Set hand forward direction - approaching from X axis in world frame
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";  // Use world frame instead of hand_frame
            vec.vector.x = 1.0;  // Approach from positive X direction (moving in +X)
            vec.vector.y = 0.0;
            vec.vector.z = 0.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        /****************************************************
    	 *               Generate Grasp Pose                *
         ****************************************************/
        {
            // Sample grasp pose
            auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "grasp_pose");
            stage->setPreGraspPose("hand_open");
            stage->setObject("object");
            stage->setAngleDelta(M_PI / 12);
            stage->setMonitoredStage(current_state_ptr);  // Hook into current state

            // Grasp frame transform for X-axis approach
            Eigen::Isometry3d grasp_frame_transform;
            // Rotate gripper to face along X axis (fingers pointing in -X direction)
            // This orients the gripper so it can grasp when approaching from +X
            Eigen::Quaterniond q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                                  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
            grasp_frame_transform.linear() = q.matrix();
            grasp_frame_transform.translation().x() = 0.02;  // Small offset for grasp
            grasp_frame_transform.translation().y() = 0.0;
            grasp_frame_transform.translation().z() = 0.0;

            // Compute IK
            auto wrapper =
                std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(8);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame(grasp_frame_transform, hand_frame);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
            grasp->insert(std::move(wrapper));

        }

        /****************************************************
    	 *         Allow Collision (hand, object)           *
         ****************************************************/
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
            stage->allowCollisions("object",
                task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
                true);
            grasp->insert(std::move(stage));
        }

        /****************************************************
    	 *                    Close Hand                    *
         ****************************************************/
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
            stage->setGroup(hand_group_name);
            stage->setGoal("hand_close");
            grasp->insert(std::move(stage));
        }

        /****************************************************
    	 *                  Attach Object                   *
         ****************************************************/
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            stage->attachObject("object", hand_frame);
            attach_object_stage = stage.get();
            grasp->insert(std::move(stage));
        }

        /****************************************************
    	 *                    Lift Object                   *
         ****************************************************/
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.1, 0.15);
            stage->setIKFrame(hand_frame);
            stage->properties().set("marker_ns", "lift_object");

            // Set upward direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        task.add(std::move(grasp));
    }
//
//	/****************************************************
//	 *                                                  *
//	 *               Move To Place                      *
//	 *                                                  *
//	 ****************************************************/
//    {
//        auto stage_move_to_place = std::make_unique<mtc::stages::Connect>("move to place",
//            mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner }});
//        stage_move_to_place->setTimeout(5.0);
//        stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
//        task.add(std::move(stage_move_to_place));
//    }
//
//	/****************************************************
//	 *                                                  *
//	 *               Place Object                       *
//	 *                                                  *
//	 ****************************************************/
//    {
//        auto place = std::make_unique<mtc::SerialContainer>("place object");
//        task.properties().exposeTo(place->properties(), { "eef", "eef", "group" });
//        place->properties().configureInitFrom(mtc::Stage::PARENT,
//                                              { "eef", "eef", "group" });
//        {
//            // Sample place pose
//            auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
//            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"ik_frame"});
//            stage->properties().set("marker_ns", "place_pose");
//            stage->setObject("object");
//
//            geometry_msgs::msg::PoseStamped target_pose_msg;
//            target_pose_msg.header.frame_id = "object";
//            //target_pose_msg.pose.position.x = -0.2;
//            target_pose_msg.pose.position.y = 0.2;
//            // target_pose_msg.pose.position.z = 0.1;
//            target_pose_msg.pose.orientation.w = 1.0;
//            stage->setPose(target_pose_msg);
//            stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage
//
//            // Compute IK
//            auto wrapper =
//                std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
//            wrapper->setMaxIKSolutions(2);
//            wrapper->setMinSolutionDistance(1.0);
//            wrapper->setIKFrame("object");
//            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
//            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
//            place->insert(std::move(wrapper));
//        }
//
//        /****************************************************
//    	 *                   Open Hand                      *
//         ****************************************************/
//        {
//            auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
//            stage->setGroup(hand_group_name);
//            stage->setGoal("hand_open");
//            place->insert(std::move(stage));
//        }
//
//        /****************************************************
//    	 *        Forbid Collision (hand, object)           *
//         ****************************************************/
//        {
//            auto stage =
//                std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
//            stage->allowCollisions("object",
//                                  task.getRobotModel()
//                                      ->getJointModelGroup(hand_group_name)
//                                      ->getLinkModelNamesWithCollisionGeometry(),
//                                  false);
//            place->insert(std::move(stage));
//        }
//
//        /****************************************************
//    	 *                Detach Object                     *
//         ****************************************************/
//        {
//            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
//            stage->detachObject("object", hand_frame);
//            place->insert(std::move(stage));
//        }
//
//        /****************************************************
//    	 *                       Retreat                    *
//         ****************************************************/
//        {
//            auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
//            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
//            stage->setMinMaxDistance(0.01, 0.3);
//            stage->setIKFrame(hand_frame);
//            stage->properties().set("marker_ns", "retreat");
//
//            // Set retreat direction
//            geometry_msgs::msg::Vector3Stamped vec;
//            vec.header.frame_id = "world";
//            // vec.vector.y = 1.0;
//            // vec.vector.x = -1.0;
//            vec.vector.z = 1.0;
//            stage->setDirection(vec);
//            place->insert(std::move(stage));
//        }
//
//          task.add(std::move(place));
//    }
//
//	/****************************************************
//	 *                                                  *
//	 *                Return Home                       *
//	 *                                                  *
//	 ****************************************************/
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setGoal("ready");
        task.add(std::move(stage));
    }

    return task;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
      executor.add_node(mtc_task_node->getNodeBaseInterface());
      executor.spin();
      executor.remove_node(mtc_task_node->getNodeBaseInterface());
    });

    mtc_task_node->setupPlanningScene();
    mtc_task_node->doTask();

    spin_thread->join();
    rclcpp::shutdown();
    return 0;
}