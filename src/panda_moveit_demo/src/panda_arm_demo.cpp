#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
  // Standard ROS 2 node bring-up — same as any rclcpp node.
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "panda_arm_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto logger = rclcpp::get_logger("panda_arm_demo");

  // MoveGroupInterface needs to spin the node in the background to receive
  // things like the current robot state and planning scene updates while we
  // block on plan()/execute() calls below. A single-threaded executor on a
  // separate thread handles this.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  // MoveGroupInterface is the object you talked to by clicking buttons in
  // RViz yesterday. "panda_arm" is the planning group name from the SRDF —
  // the same dropdown you switched from "hand" to "panda_arm".
  moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");
  // Give action-client discovery time to fully settle (goal, feedback, and
  // result channels are discovered slightly asynchronously) before the first
  // request — avoids a goal-response timeout race on the very first call.
  rclcpp::sleep_for(std::chrono::seconds(2));

  // --- Part 1: plan to a named target ---
  // "ready" is a named joint configuration defined in the SRDF, same as the
  // one you selected in the Goal State dropdown in RViz.
  move_group.setNamedTarget("ready");

  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  bool success1 = (move_group.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(logger, "Plan to 'ready': %s", success1 ? "SUCCESS" : "FAILED");

  if (success1)
  {
    move_group.execute(plan1);
    RCLCPP_INFO(logger, "Executed move to 'ready'");
  }

  // Give things a moment to settle before the next plan.
  rclcpp::sleep_for(std::chrono::seconds(1));

  // --- Part 2: plan to a pose goal ---
  // This is the harder case: we specify where we want the gripper (position
  // + orientation) instead of a named joint configuration. MoveGroupInterface
  // calls IK internally to convert this pose into joint angles before OMPL
  // plans the path.
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;  // no rotation — gripper pointing "down" in its default frame
  target_pose.position.x = 0.3;
  target_pose.position.y = 0.2;
  target_pose.position.z = 0.5;

  move_group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan2;
  bool success2 = (move_group.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(logger, "Plan to pose target: %s", success2 ? "SUCCESS" : "FAILED");

  if (success2)
  {
    move_group.execute(plan2);
    RCLCPP_INFO(logger, "Executed move to pose target");
  }
  else
  {
    RCLCPP_WARN(logger, "Pose target unreachable or planning failed — try different x/y/z");
  }

  rclcpp::shutdown();
  executor_thread.join();
  return 0;
}
