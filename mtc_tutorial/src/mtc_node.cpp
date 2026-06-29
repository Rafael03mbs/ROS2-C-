#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
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

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void doTask();
  void setupPlanningScene();

private:
  mtc::Task createTask();
  void planCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void executeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_service_;
  bool solution_available_ = false;
  
  // Cylinder positions - further from robot base for reachability
  // Cylinder 1 (blue) - initial position A
  double cyl1_x_ = 0.0;
  double cyl1_y_ = -0.18;  // Further from robot
  double cyl1_z_ = 0.02;
  
  // Cylinder 2 (green) - initial position B (further apart from cylinder 1)
  double cyl2_x_ = 0.10;   // 10cm offset in X (was 4cm)
  double cyl2_y_ = -0.18;  // Same Y as cylinder 1
  double cyl2_z_ = 0.02;
  
  // Temporary position for cylinder 1 (rotated 90 degrees from original)
  double temp_x_ = 0.18;
  double temp_y_ = 0.0;
  double temp_z_ = 0.02;
  
  // Stacking height offset
  double stack_height_ = 0.045;  // Height of cylinder + small gap
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
  // Create service to plan the MTC solution
  plan_service_ = node_->create_service<std_srvs::srv::Trigger>(
    "plan_mtc_solution",
    std::bind(&MTCTaskNode::planCallback, this, std::placeholders::_1, std::placeholders::_2));
  
  // Create service to execute the MTC solution
  execute_service_ = node_->create_service<std_srvs::srv::Trigger>(
    "execute_mtc_solution",
    std::bind(&MTCTaskNode::executeCallback, this, std::placeholders::_1, std::placeholders::_2));
  
  RCLCPP_INFO(node_->get_logger(), "===========================================");
  RCLCPP_INFO(node_->get_logger(), "MTC Node Ready! Available services:");
  RCLCPP_INFO(node_->get_logger(), "  1. ros2 service call /plan_mtc_solution std_srvs/srv/Trigger");
  RCLCPP_INFO(node_->get_logger(), "  2. ros2 service call /execute_mtc_solution std_srvs/srv/Trigger");
  RCLCPP_INFO(node_->get_logger(), "===========================================");
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
  moveit::planning_interface::PlanningSceneInterface psi;

  // Cylinder 1 (Blue) - at position A
  moveit_msgs::msg::CollisionObject cylinder1;
  cylinder1.id = "cylinder1";
  cylinder1.header.frame_id = "world";
  cylinder1.primitives.resize(1);
  cylinder1.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  cylinder1.primitives[0].dimensions = { 0.04, 0.01 };  // height=0.04m, radius=0.01m
  cylinder1.pose.position.x = cyl1_x_;
  cylinder1.pose.position.y = cyl1_y_;
  cylinder1.pose.position.z = cyl1_z_;
  cylinder1.pose.orientation.w = 1.0;

  std_msgs::msg::ColorRGBA blue;
  blue.r = 0.0; blue.g = 0.0; blue.b = 1.0; blue.a = 1.0;
  psi.applyCollisionObject(cylinder1, blue);

  RCLCPP_INFO(LOGGER, "Cylinder 1 (blue) at (%.2f, %.2f, %.2f)", cyl1_x_, cyl1_y_, cyl1_z_);

  // Cylinder 2 (Green) - at position B
  moveit_msgs::msg::CollisionObject cylinder2;
  cylinder2.id = "cylinder2";
  cylinder2.header.frame_id = "world";
  cylinder2.primitives.resize(1);
  cylinder2.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  cylinder2.primitives[0].dimensions = { 0.04, 0.01 };
  cylinder2.pose.position.x = cyl2_x_;
  cylinder2.pose.position.y = cyl2_y_;
  cylinder2.pose.position.z = cyl2_z_;
  cylinder2.pose.orientation.w = 1.0;

  std_msgs::msg::ColorRGBA green;
  green.r = 0.0; green.g = 1.0; green.b = 0.0; green.a = 1.0;
  psi.applyCollisionObject(cylinder2, green);

  RCLCPP_INFO(LOGGER, "Cylinder 2 (green) at (%.2f, %.2f, %.2f)", cyl2_x_, cyl2_y_, cyl2_z_);
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  RCLCPP_INFO(LOGGER, "Initializing task...");
  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "InitStageException: " << e);
    return;
  }
  RCLCPP_INFO(LOGGER, "Task initialized.");
  
  task_.enableIntrospection(true);
  rclcpp::sleep_for(std::chrono::seconds(2));

  RCLCPP_INFO(LOGGER, "Planning two-cylinder reordering...");
  RCLCPP_INFO(LOGGER, "Planning two-cylinder reordering...");
  if (task_.plan(10))
  {
    RCLCPP_INFO(LOGGER, "Planning SUCCESS!");
    task_.introspection().publishSolution(*task_.solutions().front());
    
    RCLCPP_INFO(LOGGER, "Executing solution automatically...");
    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR(LOGGER, "Execution failed! Error code: %d", result.val);
    }
    else
    {
      RCLCPP_INFO(LOGGER, "Execution SUCCESS!");
    }
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning FAILED!");
    
    // Find the failed stage
    if (task_.solutions().empty())
    {
       // If no solutions, we need to inspect the task structure to find where it failed
       // This is a simplified approach: we just print the state which usually shows the error
       task_.printState();
       
       // To be more specific, we can iterate containers, but printState is usually enough for logs
       // Let's also print a clear specific message if we can find one
       // NOTE: Deep inspection is complex, relying on printState for now as it shows the stage statuses
    }
  }
}

void MTCTaskNode::planCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(LOGGER, "Plan service called - starting planning...");
  
  task_ = createTask();

  RCLCPP_INFO(LOGGER, "Initializing task...");
  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "InitStageException: " << e);
    response->success = false;
    response->message = "Task initialization failed!";
    return;
  }
  RCLCPP_INFO(LOGGER, "Task initialized.");
  
  task_.enableIntrospection(true);

  RCLCPP_INFO(LOGGER, "Planning two-cylinder reordering...");
  if (!task_.plan(10))
  {
    RCLCPP_ERROR(LOGGER, "Planning FAILED!");
    task_.printState();
    response->success = false;
    response->message = "Planning failed!";
    return;
  }
  
  solution_available_ = true;
  task_.introspection().publishSolution(*task_.solutions().front());
  
  RCLCPP_INFO(LOGGER, "===========================================");
  RCLCPP_INFO(LOGGER, "Planning SUCCESS! Solution published to RViz.");
  RCLCPP_INFO(LOGGER, "===========================================");
  RCLCPP_INFO(LOGGER, "To EXECUTE, call:");
  RCLCPP_INFO(LOGGER, "  ros2 service call /execute_mtc_solution std_srvs/srv/Trigger");
  RCLCPP_INFO(LOGGER, "===========================================");
  
  response->success = true;
  response->message = "Planning succeeded! Solution published to RViz. Call /execute_mtc_solution to execute.";
}

void MTCTaskNode::executeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!solution_available_)
  {
    response->success = false;
    response->message = "No solution available. Planning may have failed.";
    RCLCPP_WARN(LOGGER, "Execute called but no solution available!");
    return;
  }

  RCLCPP_INFO(LOGGER, "Executing MTC solution...");
  
  auto result = task_.execute(*task_.solutions().front());
  if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    response->success = true;
    response->message = "MTC solution executed successfully!";
    RCLCPP_INFO(LOGGER, "Execution SUCCESS!");
  }
  else
  {
    response->success = false;
    response->message = "Execution failed with error code: " + std::to_string(result.val);
    RCLCPP_ERROR(LOGGER, "Execution FAILED with error code: %d", result.val);
  }
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("two cylinder reorder");
  task.loadRobotModel(node_);

  const auto& arm = "arm";
  const auto& gripper = "gripper";
  const auto& ik_frame = "tool_tip";

  task.setProperty("group", arm);
  task.setProperty("eef", "end_effector");
  task.setProperty("ik_frame", ik_frame);

  auto interpolation = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  
  // OMPL planner for motions that need collision-free path planning
  auto ompl_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  ompl_planner->setProperty("goal_joint_tolerance", 1e-4);
  
  auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian->setMaxVelocityScalingFactor(1.0);
  cartesian->setMaxAccelerationScalingFactor(1.0);
  cartesian->setStepSize(0.005);
  cartesian->setMinFraction(0.8);

  // ========================================
  // STAGE: Current state
  // ========================================
  {
    auto stage = std::make_unique<mtc::stages::CurrentState>("current");
    task.add(std::move(stage));
  }

  // ========================================
  // STAGE: Open gripper initially
  // ========================================
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation);
    stage->setGroup(gripper);
    stage->setGoal("open");
    task.add(std::move(stage));
  }

  // ========================================
  // PICK CYLINDER 1 - Move to temporary position
  // ========================================
  RCLCPP_INFO(LOGGER, "Adding stages: Pick Cylinder 1");

  // Move to pre-grasp position for cylinder 1
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("pre-grasp cylinder1", interpolation);
    stage->setGroup(arm);
    std::map<std::string, double> joints;
    // Position above cylinder 1 at (0.0, -0.18) - arm extended further
    joints["Rotation"] = 0.0;     // Straight ahead
    joints["Pitch"] = 0.3;        // Lean forward more
    joints["Elbow"] = 0.6;        // Elbow bent
    joints["Wrist_Pitch"] = 0.8;  // Wrist angle
    joints["Wrist_Roll"] = 0.0;
    stage->setGoal(joints);
    task.add(std::move(stage));
  }

  // Allow collision with cylinder1
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision cylinder1");
    stage->allowCollisions("cylinder1",
                           task.getRobotModel()
                               ->getJointModelGroup(gripper)
                               ->getLinkModelNamesWithCollisionGeometry(),
                           true);
    task.add(std::move(stage));
  }

  // Close gripper to grasp cylinder1
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("grasp cylinder1", interpolation);
    stage->setGroup(gripper);
    stage->setGoal("close");
    task.add(std::move(stage));
  }

  // Attach cylinder1
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach cylinder1");
    stage->attachObject("cylinder1", ik_frame);
    task.add(std::move(stage));
  }

  // Lift cylinder1
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lift cylinder1", cartesian);
    stage->setGroup(arm);
    stage->setMinMaxDistance(0.03, 0.10);
    stage->setIKFrame(ik_frame);
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  // Move cylinder1 to temporary position (rotated ~90 degrees) - using OMPL for obstacle avoidance
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move cylinder1 to temp", ompl_planner);
    stage->setGroup(arm);
    std::map<std::string, double> joints;
    // Temporary position - rotate base 90 degrees, arm extended
    joints["Rotation"] = 1.57;   // ~90 degrees
    joints["Pitch"] = 0.3;       // Forward lean
    joints["Elbow"] = 0.6;
    joints["Wrist_Pitch"] = 0.8;
    joints["Wrist_Roll"] = 0.0;
    stage->setGoal(joints);
    task.add(std::move(stage));
  }

  // Lower cylinder1 at temp position
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lower cylinder1 at temp", cartesian);
    stage->setGroup(arm);
    stage->setMinMaxDistance(0.01, 0.05);
    stage->setIKFrame(ik_frame);
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = -1.0;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  // Release cylinder1
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("release cylinder1", interpolation);
    stage->setGroup(gripper);
    stage->setGoal("open");
    task.add(std::move(stage));
  }

  // Detach cylinder1
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach cylinder1");
    stage->detachObject("cylinder1", ik_frame);
    task.add(std::move(stage));
  }

  // Forbid collision with cylinder1
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision cylinder1");
    stage->allowCollisions("cylinder1",
                           task.getRobotModel()
                               ->getJointModelGroup(gripper)
                               ->getLinkModelNamesWithCollisionGeometry(),
                           false);
    task.add(std::move(stage));
  }

  // Retreat from cylinder1
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat from cylinder1", cartesian);
    stage->setGroup(arm);
    stage->setMinMaxDistance(0.03, 0.10);
    stage->setIKFrame(ik_frame);
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  // ========================================
  // PICK CYLINDER 2 - Move to Cylinder 1's original position
  // ========================================
  RCLCPP_INFO(LOGGER, "Adding stages: Pick Cylinder 2");

  // First go to a safe intermediate position to avoid collision with cylinder1 at temp - using OMPL
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("intermediate home", ompl_planner);
    stage->setGroup(arm);
    stage->setGoal("ready");
    task.add(std::move(stage));
  }

  // Allow collision with cylinder2 BEFORE any motion towards it
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision cylinder2");
    stage->allowCollisions("cylinder2",
                           task.getRobotModel()
                               ->getJointModelGroup(gripper)
                               ->getLinkModelNamesWithCollisionGeometry(),
                           true);
    task.add(std::move(stage));
  }

  // Move to pre-grasp position for cylinder 2 (using OMPL for collision-free path)
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("pre-grasp cylinder2", ompl_planner);
    stage->setGroup(arm);
    std::map<std::string, double> joints;
    // Position above cylinder 2 at (0.10, -0.18) - more rotation needed for 10cm offset
    joints["Rotation"] = -0.5;   // More rotation for 10cm offset
    joints["Pitch"] = 0.3;       // Forward lean
    joints["Elbow"] = 0.6;
    joints["Wrist_Pitch"] = 0.8;
    joints["Wrist_Roll"] = 0.0;
    stage->setGoal(joints);
    task.add(std::move(stage));
  }

  // Close gripper to grasp cylinder2
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("grasp cylinder2", interpolation);
    stage->setGroup(gripper);
    stage->setGoal("close");
    task.add(std::move(stage));
  }

  // Attach cylinder2
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach cylinder2");
    stage->attachObject("cylinder2", ik_frame);
    task.add(std::move(stage));
  }

  // Lift cylinder2
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lift cylinder2", cartesian);
    stage->setGroup(arm);
    stage->setMinMaxDistance(0.03, 0.10);
    stage->setIKFrame(ik_frame);
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  // Move cylinder2 to cylinder1's original position - using OMPL for obstacle avoidance
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move cylinder2 to pos A", ompl_planner);
    stage->setGroup(arm);
    std::map<std::string, double> joints;
    // Back to cylinder 1's original position (0.0, -0.18)
    joints["Rotation"] = 0.0;
    joints["Pitch"] = 0.3;
    joints["Elbow"] = 0.6;
    joints["Wrist_Pitch"] = 0.8;
    joints["Wrist_Roll"] = 0.0;
    stage->setGoal(joints);
    task.add(std::move(stage));
  }

  // Lower cylinder2 at position A
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lower cylinder2 at pos A", cartesian);
    stage->setGroup(arm);
    stage->setMinMaxDistance(0.01, 0.05);
    stage->setIKFrame(ik_frame);
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = -1.0;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  // Release cylinder2
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("release cylinder2", interpolation);
    stage->setGroup(gripper);
    stage->setGoal("open");
    task.add(std::move(stage));
  }

  // Detach cylinder2
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach cylinder2");
    stage->detachObject("cylinder2", ik_frame);
    task.add(std::move(stage));
  }

  // Forbid collision with cylinder2
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision cylinder2");
    stage->allowCollisions("cylinder2",
                           task.getRobotModel()
                               ->getJointModelGroup(gripper)
                               ->getLinkModelNamesWithCollisionGeometry(),
                           false);
    task.add(std::move(stage));
  }

  // Retreat from cylinder2
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat from cylinder2", cartesian);
    stage->setGroup(arm);
    stage->setMinMaxDistance(0.03, 0.10);
    stage->setIKFrame(ik_frame);
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  // ========================================
  // PICK CYLINDER 1 AGAIN - Stack on Cylinder 2
  // ========================================
  RCLCPP_INFO(LOGGER, "Adding stages: Pick Cylinder 1 again and stack");

  // Go to safe intermediate position first - using OMPL for obstacle avoidance
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("intermediate home 2", ompl_planner);
    stage->setGroup(arm);
    stage->setGoal("ready");
    task.add(std::move(stage));
  }

  // Allow collision with cylinder1 BEFORE approaching
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision cylinder1 again");
    stage->allowCollisions("cylinder1",
                           task.getRobotModel()
                               ->getJointModelGroup(gripper)
                               ->getLinkModelNamesWithCollisionGeometry(),
                           true);
    task.add(std::move(stage));
  }

  // Move to pre-grasp position for cylinder 1 at temp location (using OMPL)
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("pre-grasp cylinder1 at temp", ompl_planner);
    stage->setGroup(arm);
    std::map<std::string, double> joints;
    // Position at temporary location (90 degrees rotated)
    joints["Rotation"] = 1.57;
    joints["Pitch"] = 0.3;
    joints["Elbow"] = 0.6;
    joints["Wrist_Pitch"] = 0.8;
    joints["Wrist_Roll"] = 0.0;
    stage->setGoal(joints);
    task.add(std::move(stage));
  }

  // Close gripper to grasp cylinder1
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("grasp cylinder1 again", interpolation);
    stage->setGroup(gripper);
    stage->setGoal("close");
    task.add(std::move(stage));
  }

  // Attach cylinder1
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach cylinder1 again");
    stage->attachObject("cylinder1", ik_frame);
    task.add(std::move(stage));
  }

  // Lift cylinder1
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lift cylinder1 for stacking", cartesian);
    stage->setGroup(arm);
    stage->setMinMaxDistance(0.03, 0.10);
    stage->setIKFrame(ik_frame);
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  // Allow collision between cylinder1 and cylinder2 for stacking
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow stacking collision");
    stage->allowCollisions("cylinder1", "cylinder2", true);
    task.add(std::move(stage));
  }

  // Also allow gripper collision with cylinder2 for stacking
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow gripper-cylinder2 collision");
    stage->allowCollisions("cylinder2",
                           task.getRobotModel()
                               ->getJointModelGroup(gripper)
                               ->getLinkModelNamesWithCollisionGeometry(),
                           true);
    task.add(std::move(stage));
  }

  // Move cylinder1 to stack position (above cylinder2 at original position A) - using OMPL
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move cylinder1 to stack", ompl_planner);
    stage->setGroup(arm);
    std::map<std::string, double> joints;
    // Position above cylinder 2 (now at position A) - higher for stacking
    joints["Rotation"] = -0.5;
    joints["Pitch"] = 0.2;     // Slightly higher for stacking
    joints["Elbow"] = 0.5;
    joints["Wrist_Pitch"] = 0.9;
    joints["Wrist_Roll"] = 0.0;
    stage->setGoal(joints);
    task.add(std::move(stage));
  }

  // Lower cylinder1 onto cylinder2 (less distance since we're stacking)
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lower cylinder1 onto cylinder2", cartesian);
    stage->setGroup(arm);
    stage->setMinMaxDistance(0.01, 0.03);  // Smaller distance for precise stacking
    stage->setIKFrame(ik_frame);
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = -1.0;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  // Release cylinder1
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("release cylinder1 final", interpolation);
    stage->setGroup(gripper);
    stage->setGoal("open");
    task.add(std::move(stage));
  }

  // Detach cylinder1
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach cylinder1 final");
    stage->detachObject("cylinder1", ik_frame);
    task.add(std::move(stage));
  }

  // Forbid collision with cylinder1
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision cylinder1 final");
    stage->allowCollisions("cylinder1",
                           task.getRobotModel()
                               ->getJointModelGroup(gripper)
                               ->getLinkModelNamesWithCollisionGeometry(),
                           false);
    task.add(std::move(stage));
  }

  // Retreat from stacked cylinders
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("final retreat", cartesian);
    stage->setGroup(arm);
    stage->setMinMaxDistance(0.03, 0.10);
    stage->setIKFrame(ik_frame);
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  // ========================================
  // RETURN HOME - using OMPL for obstacle avoidance
  // ========================================
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", ompl_planner);
    stage->setGroup(arm);
    stage->setGoal("ready");
    task.add(std::move(stage));
  }

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &node]() {
    executor.add_node(node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(node->getNodeBaseInterface());
  });

  node->setupPlanningScene();
  
  // Node now waits for service calls instead of running automatically
  // Call: ros2 service call /plan_mtc_solution std_srvs/srv/Trigger
  // Then: ros2 service call /execute_mtc_solution std_srvs/srv/Trigger
  
  // Create a thread to run the task so it doesn't block the executor
  std::thread task_thread([node](){
      rclcpp::sleep_for(std::chrono::seconds(2)); // Wait for RViz to connect
      node->doTask();
  });
  task_thread.detach();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
