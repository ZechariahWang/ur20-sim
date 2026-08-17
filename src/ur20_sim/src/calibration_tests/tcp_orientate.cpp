#include "calibration_tests/tcp_orientate.hpp"

#include <memory>

TcpOrientate::TcpOrientate()
    : Node("tcp_orientate",
          rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {}

TcpOrientate::~TcpOrientate() {
  if (spinner_.joinable()) {
    executor_.cancel();
    spinner_.join();
  }
}

bool TcpOrientate::run() {

  park_joints_ = get_parameter_or<std::vector<double>>("park_joints", {});

  executor_.add_node(shared_from_this());
  spinner_ = std::thread([this]() { executor_.spin(); });

  std::string group_name = get_parameter_or<std::string>("planning_group", "ur_manipulator");
  double wiggle = get_parameter_or<double>("wiggle_rad", 0.2);
  // Test speed: fraction (0-1] of maximum joint velocity/acceleration.
  double test_speed = get_parameter_or<double>("test_speed", 0.03);

  moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), group_name);
  move_group.setMaxVelocityScalingFactor(test_speed);
  move_group.setMaxAccelerationScalingFactor(test_speed);
  RCLCPP_INFO(get_logger(), "Test speed scaling: %.2f", test_speed);

  rclcpp::sleep_for(std::chrono::seconds(1));

  // First move to the parked pose so the test always runs from the
  // same place.
  if (park_joints_.size() == 6) {
    if (!moveTo(move_group, park_joints_, "move to park position")) { return false; }
  }

  moveit::core::RobotStatePtr current = move_group.getCurrentState(10.0);
  const moveit::core::JointModelGroup *group = current->getJointModelGroup(group_name);
  std::vector<double> joints;
  current->copyJointGroupPositions(group, joints);

  RCLCPP_INFO(get_logger(), "Current joints: %.2f %.2f %.2f %.2f %.2f %.2f",
              joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);

  // Move wrist_3 out by the wiggle angle, then back to where it was.
  std::vector<double> out = joints;
  out[5] = out[5] + wiggle;
  if (!moveTo(move_group, out, "wiggle out")) { return false; }
  if (!moveTo(move_group, joints, "wiggle back")) { return false; }

  RCLCPP_INFO(get_logger(), "Test complete. The robot moved and returned.");
  return true;
}

bool TcpOrientate::moveTo(moveit::planning_interface::MoveGroupInterface &move_group, const std::vector<double> &target, const std::string &label) {
  move_group.setJointValueTarget(target);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool planned = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!planned) {
    RCLCPP_ERROR(get_logger(), "Planning failed: %s", label.c_str());
    return false;
  }
  RCLCPP_INFO(get_logger(), "Moving: %s...", label.c_str());
  bool executed = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!executed) {
    RCLCPP_ERROR(get_logger(), "Execution failed: %s", label.c_str());
    return false;
  }
  return true;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TcpOrientate>();
  bool ok = node->run();
  rclcpp::shutdown();
  if (ok) { return 0; }
  return 1;
}
