#include "calibration_tests/axis_test.hpp"

#include <cmath>
#include <memory>

#include "utils.hpp"

AxisTest::AxisTest()
    : Node("axis_test",
          rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {}

AxisTest::~AxisTest() {
  stopSpinner();
}

bool AxisTest::run() {
  loadParameters();
  if (!setup()) { return false; }
  bool ok = doMovement();
  if (ok) {
    RCLCPP_INFO(get_logger(), "Axis test complete.");
  } else {
    RCLCPP_INFO(get_logger(), "Axis test finished with errors.");
  }
  return ok;
}

void AxisTest::loadParameters() {
  planning_group_ = get_parameter_or<std::string>("planning_group", "ur_manipulator");
  test_speed_ = get_parameter_or<double>("test_speed", 0.03);
  center_x_ = get_parameter_or<double>("center_x", 1.0);
  center_y_ = get_parameter_or<double>("center_y", 0.0);
  center_z_ = get_parameter_or<double>("center_z", 0.35);
  y_travel_ = get_parameter_or<double>("y_travel", 0.4);
  z_travel_ = get_parameter_or<double>("z_travel", 0.25);
  rotation_rad_ = get_parameter_or<double>("rotation_rad", 1.57);
  eef_step_ = get_parameter_or<double>("eef_step", 0.01);
  park_joints_ = get_parameter_or<std::vector<double>>("park_joints", {});
}

bool AxisTest::setup() {
  executor_.add_node(shared_from_this());
  spinner_ = std::thread([this]() { executor_.spin(); });

  move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group_);
  move_group_->setMaxVelocityScalingFactor(test_speed_);
  move_group_->setMaxAccelerationScalingFactor(test_speed_);
  RCLCPP_INFO(get_logger(), "Axis test speed scaling: %.2f", test_speed_);

  rclcpp::sleep_for(std::chrono::seconds(1));
  return true;
}

bool AxisTest::doMovement() {

  // Start from the park pose so every run is identical.
  if (park_joints_.size() == 6) {
    if (!utils::moveToJoints(*move_group_, get_logger(), park_joints_, "move to park position")) { return false; }
  }

  geometry_msgs::msg::Quaternion down = utils::pitchQuaternion(M_PI);
  geometry_msgs::msg::Pose center = utils::makePose(center_x_, center_y_, center_z_, down);

  if (!utils::moveToPose(*move_group_, get_logger(), center, "move to center")) { return false; }

  // Left to right sweep along the y axis.
  if (!utils::sweepTo(*move_group_, get_logger(), eef_step_, test_speed_, test_speed_,
                      utils::makePose(center_x_, center_y_ + y_travel_, center_z_, down), "sweep to left")) { return false; }
  if (!utils::sweepTo(*move_group_, get_logger(), eef_step_, test_speed_, test_speed_,
                      utils::makePose(center_x_, center_y_ - y_travel_, center_z_, down), "sweep left to right")) { return false; }
  if (!utils::sweepTo(*move_group_, get_logger(), eef_step_, test_speed_, test_speed_,
                      center, "return to center")) { return false; }

  // Up and down sweep along the z axis.
  if (!utils::sweepTo(*move_group_, get_logger(), eef_step_, test_speed_, test_speed_,
                      utils::makePose(center_x_, center_y_, center_z_ + z_travel_, down), "sweep up")) { return false; }
  if (!utils::sweepTo(*move_group_, get_logger(), eef_step_, test_speed_, test_speed_,
                      utils::makePose(center_x_, center_y_, center_z_ - z_travel_, down), "sweep up to down")) { return false; }
  if (!utils::sweepTo(*move_group_, get_logger(), eef_step_, test_speed_, test_speed_,
                      center, "return to center")) { return false; }

  // Rotate the TCP in place: turn wrist_3 out and back.
  moveit::core::RobotStatePtr current = move_group_->getCurrentState(10.0);
  const moveit::core::JointModelGroup *group = current->getJointModelGroup(planning_group_);
  std::vector<double> joints;
  current->copyJointGroupPositions(group, joints);

  std::vector<double> rotated = joints;
  rotated[5] = rotated[5] + rotation_rad_;
  if (!utils::moveToJoints(*move_group_, get_logger(), rotated, "rotate TCP")) { return false; }
  if (!utils::moveToJoints(*move_group_, get_logger(), joints, "rotate TCP back")) { return false; }

  // Finish back in the resting (park) pose.
  if (park_joints_.size() == 6) {
    return utils::moveToJoints(*move_group_, get_logger(), park_joints_, "return to park");
  }
  return true;
}

void AxisTest::stopSpinner() {
  if (spinner_.joinable()) {
    executor_.cancel();
    spinner_.join();
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AxisTest>();
  bool ok = node->run();
  rclcpp::shutdown();
  if (ok) { return 0; }
  return 1;
}
