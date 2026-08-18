#include "jog_to_rest.hpp"

#include <memory>

#include "utils.hpp"

JogToRest::JogToRest()
    : Node("jog_to_rest",
          rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {}

JogToRest::~JogToRest() {
  stopSpinner();
}

bool JogToRest::run() {
  loadParameters();

  if (rest_joints_.size() != 6) {
    RCLCPP_ERROR(get_logger(), "jog_to_rest needs rest_joints (6 values) in rest.yaml.");
    return false;
  }

  if (!setup()) { return false; }

  // Log where the flange and camera tip will end up (base frame).
  geometry_msgs::msg::Pose flange_pose = utils::poseFromJoints(*move_group_, rest_joints_);
  geometry_msgs::msg::Pose camera_pose = utils::cameraPoseFromFlangePose(flange_pose, camera_length_);
  RCLCPP_INFO(get_logger(), "Rest pose: flange (%.2f, %.2f, %.2f), camera tip (%.2f, %.2f, %.2f).",
              flange_pose.position.x, flange_pose.position.y, flange_pose.position.z,
              camera_pose.position.x, camera_pose.position.y, camera_pose.position.z);

  bool ok = utils::moveToNearestJoints(*move_group_, get_logger(), rest_joints_, "jog to rest");
  if (ok) {
    RCLCPP_INFO(get_logger(), "Arm is resting. Safe to power the robot down.");
  } else {
    RCLCPP_ERROR(get_logger(), "Could not reach the rest pose.");
  }
  return ok;
}

void JogToRest::loadParameters() {
  planning_group_ = get_parameter_or<std::string>("planning_group", "ur_manipulator");
  velocity_scaling_ = get_parameter_or<double>("velocity_scaling", 0.05);
  acceleration_scaling_ = get_parameter_or<double>("acceleration_scaling", 0.1);
  camera_length_ = get_parameter_or<double>("camera_length", 0.22);
  rest_joints_ = get_parameter_or<std::vector<double>>("rest_joints", {});
}

bool JogToRest::setup() {

  executor_.add_node(shared_from_this());
  spinner_ = std::thread([this]() { executor_.spin(); });

  move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group_);
  move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);

  rclcpp::sleep_for(std::chrono::seconds(1));
  return true;
}

void JogToRest::stopSpinner() {
  if (spinner_.joinable()) {
    executor_.cancel();
    spinner_.join();
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JogToRest>();
  bool ok = node->run();
  rclcpp::shutdown();
  if (ok) { return 0; }
  return 1;
}
