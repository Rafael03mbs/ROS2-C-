#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/msg/display_trajectory.hpp>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "base", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create a closure for updating the text in rviz
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                     rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
       jmg = move_group_interface.getRobotModel()->getJointModelGroup("arm")](
          auto const trajectory) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
      };

  // Create publisher to clear ghost animation
  auto display_publisher = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", 1);
      
  auto const clear_ghost = [&display_publisher, &logger] {
      RCLCPP_INFO(logger, "Clearing ghost animation...");
      moveit_msgs::msg::DisplayTrajectory msg;
      msg.model_id = "so101_new_calib";
      // Publish multiple times to ensure receipt
      display_publisher->publish(msg);
      rclcpp::sleep_for(std::chrono::milliseconds(200));
      display_publisher->publish(msg);
      rclcpp::sleep_for(std::chrono::milliseconds(200));
      display_publisher->publish(msg);
  };

  auto const show_ghost = [&display_publisher](auto const& plan) {
      moveit_msgs::msg::DisplayTrajectory msg;
      msg.model_id = "so101_new_calib";
      msg.trajectory.push_back(plan.trajectory_);
      display_publisher->publish(msg);
  };

  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success) {
    // Show trajectory preview
    draw_trajectory_tool_path(plan.trajectory_);
    show_ghost(plan);
    draw_title("Plan Successful - Review Path");
    moveit_visual_tools.trigger();
    
    // Wait for user to decide to execute
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");

    // Clear visualization
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.trigger();
    
    // Clear ghost animation
    clear_ghost();

    draw_title("Executing");
    moveit_visual_tools.trigger();

    // Execute
    move_group_interface.execute(plan);
    
    // Final cleanup
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.trigger();

  } else {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Create collision object for the robot to avoid
  auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.5;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.2;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.25;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);

  // Visualize the collision object
  draw_title("Obstacle Added");
  moveit_visual_tools.publishCollisionBlock(
      collision_object.primitive_poses[0],
      "box1",
      rviz_visual_tools::GREEN);
  moveit_visual_tools.trigger();
  
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan around obstacle");
  moveit_visual_tools.trigger();

  // Now plan to a new position with the obstacle in place
  auto const target_pose2 = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = 0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose2);

  // Create a plan to that target pose
  draw_title("Planning around obstacle");
  moveit_visual_tools.trigger();
  auto const [success2, plan2] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success2) {
    // Show trajectory preview
    draw_trajectory_tool_path(plan2.trajectory_);
    show_ghost(plan2);
    draw_title("Plan Successful - Review Path");
    moveit_visual_tools.trigger();
    
    // Wait for user to decide to execute
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");

    // Clear visualization
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.trigger();
    
    // Clear ghost animation
    clear_ghost();

    draw_title("Executing");
    moveit_visual_tools.trigger();
    
    // Execute
    move_group_interface.execute(plan2);
    
    // Final cleanup
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.trigger();

  } else {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning around obstacle failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
