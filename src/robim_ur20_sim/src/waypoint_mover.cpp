// Moves the UR20 through a list of named joint-space waypoints using MoveIt 2.
// Waypoints are defined in config/waypoints.yaml.

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "waypoint_mover",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const auto logger = node->get_logger();

  // MoveGroupInterface needs the node spinning for its action clients and TF.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  const auto planning_group =
    node->get_parameter_or<std::string>("planning_group", "ur_manipulator");
  const double velocity_scaling = node->get_parameter_or<double>("velocity_scaling", 0.3);
  const double accel_scaling = node->get_parameter_or<double>("acceleration_scaling", 0.3);
  const bool cycle = node->get_parameter_or<bool>("cycle", false);

  const auto joint_names = node->get_parameter_or<std::vector<std::string>>(
    "joint_names",
    {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
     "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"});

  std::vector<std::string> waypoint_names;
  if (!node->get_parameter("waypoint_names", waypoint_names) || waypoint_names.empty()) {
    RCLCPP_ERROR(logger, "No 'waypoint_names' parameter set — nothing to do.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);
  move_group.setMaxVelocityScalingFactor(velocity_scaling);
  move_group.setMaxAccelerationScalingFactor(accel_scaling);

  RCLCPP_INFO(
    logger, "Planning group '%s' ready. Visiting %zu waypoint(s)%s.",
    planning_group.c_str(), waypoint_names.size(), cycle ? " in a loop" : "");

  bool all_ok = true;
  do {
    for (const auto & name : waypoint_names) {
      if (!rclcpp::ok()) {
        break;
      }

      std::vector<double> positions;
      if (!node->get_parameter("waypoints." + name, positions)) {
        RCLCPP_ERROR(logger, "Waypoint '%s' listed but not defined under 'waypoints.'", name.c_str());
        all_ok = false;
        continue;
      }
      if (positions.size() != joint_names.size()) {
        RCLCPP_ERROR(
          logger, "Waypoint '%s' has %zu values, expected %zu.",
          name.c_str(), positions.size(), joint_names.size());
        all_ok = false;
        continue;
      }

      std::map<std::string, double> target;
      for (size_t i = 0; i < joint_names.size(); ++i) {
        target[joint_names[i]] = positions[i];
      }

      move_group.setJointValueTarget(target);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "Planning to waypoint '%s' failed.", name.c_str());
        all_ok = false;
        continue;
      }

      RCLCPP_INFO(logger, "Moving to waypoint '%s'...", name.c_str());
      if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "Execution of waypoint '%s' failed.", name.c_str());
        all_ok = false;
        continue;
      }
      RCLCPP_INFO(logger, "Reached waypoint '%s'.", name.c_str());
    }
  } while (cycle && rclcpp::ok());

  RCLCPP_INFO(logger, all_ok ? "All waypoints reached." : "Finished with errors.");
  rclcpp::shutdown();
  spinner.join();
  return all_ok ? 0 : 1;
}
