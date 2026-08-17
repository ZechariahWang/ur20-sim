#include "axis_test.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

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
    if (!moveToJoints(park_joints_, "move to park position")) { return false; }
  }

  geometry_msgs::msg::Quaternion down = pitchQuaternion(M_PI);
  geometry_msgs::msg::Pose center = makePose(center_x_, center_y_, center_z_, down);

  if (!moveToPose(center, "move to center")) { return false; }

  // Left to right sweep along the y axis.
  if (!sweepTo(makePose(center_x_, center_y_ + y_travel_, center_z_, down), "sweep to left")) { return false; }
  if (!sweepTo(makePose(center_x_, center_y_ - y_travel_, center_z_, down), "sweep left to right")) { return false; }
  if (!sweepTo(center, "return to center")) { return false; }

  // Up and down sweep along the z axis.
  if (!sweepTo(makePose(center_x_, center_y_, center_z_ + z_travel_, down), "sweep up")) { return false; }
  if (!sweepTo(makePose(center_x_, center_y_, center_z_ - z_travel_, down), "sweep up to down")) { return false; }
  if (!sweepTo(center, "return to center")) { return false; }

  // Rotate the TCP in place: turn wrist_3 out and back.
  moveit::core::RobotStatePtr current = move_group_->getCurrentState(10.0);
  const moveit::core::JointModelGroup *group = current->getJointModelGroup(planning_group_);
  std::vector<double> joints;
  current->copyJointGroupPositions(group, joints);

  std::vector<double> rotated = joints;
  rotated[5] = rotated[5] + rotation_rad_;
  if (!moveToJoints(rotated, "rotate TCP")) { return false; }
  return moveToJoints(joints, "rotate TCP back");
}

bool AxisTest::moveToPose(const geometry_msgs::msg::Pose &pose, const std::string &label) {

  moveit::core::RobotStatePtr current = move_group_->getCurrentState(10.0);
  const moveit::core::JointModelGroup *group = current->getJointModelGroup(planning_group_);
  std::vector<double> current_joints;
  current->copyJointGroupPositions(group, current_joints);

  // Solve IK from several seeds, wrap each solution near the current
  // joints, and try the closest solutions first.
  std::vector<IkCandidate> candidates;
  moveit::core::RobotState state(*current);
  for (int attempt = 0; attempt < 20; attempt++) {
    if (attempt > 0) {
      state.setToRandomPositions(group);
    }
    bool found = state.setFromIK(group, pose, 0.1);
    if (!found) { continue; }

    std::vector<double> solution;
    state.copyJointGroupPositions(group, solution);
    wrapToNearest(solution, current_joints, group);

    double distance = 0.0;
    for (size_t i = 0; i < solution.size(); i++) {
      distance = distance + std::abs(solution[i] - current_joints[i]);
    }
    if (isDuplicate(solution, candidates)) { continue; }
    candidates.push_back({distance, solution});
  }
  if (candidates.empty()) {
    RCLCPP_ERROR(get_logger(), "No reachable IK solution: %s", label.c_str());
    return false;
  }

  std::sort(candidates.begin(), candidates.end(), closerCandidate);
  for (size_t i = 0; i < candidates.size(); i++) {
    IkCandidate candidate = candidates[i];
    move_group_->setJointValueTarget(candidate.joints);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!planned) {
      RCLCPP_WARN(get_logger(), "Candidate %zu rejected for %s, trying next.", i, label.c_str());
      continue;
    }

    RCLCPP_INFO(get_logger(), "Moving: %s (joint distance %.2f rad)...", label.c_str(), candidate.distance);
    bool executed = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!executed) {
      RCLCPP_ERROR(get_logger(), "Execution failed: %s", label.c_str());
      return false;
    }
    return true;
  }

  RCLCPP_ERROR(get_logger(), "Planning failed for every IK solution: %s", label.c_str());
  return false;
}

bool AxisTest::moveToJoints(const std::vector<double> &target, const std::string &label) {
  move_group_->setJointValueTarget(target);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool planned = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!planned) {
    RCLCPP_ERROR(get_logger(), "Planning failed: %s", label.c_str());
    return false;
  }
  RCLCPP_INFO(get_logger(), "Moving: %s...", label.c_str());
  bool executed = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!executed) {
    RCLCPP_ERROR(get_logger(), "Execution failed: %s", label.c_str());
    return false;
  }
  return true;
}

bool AxisTest::sweepTo(const geometry_msgs::msg::Pose &pose, const std::string &label) {

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(pose);
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group_->computeCartesianPath(waypoints, eef_step_, 0.0, trajectory);
  if (fraction < 0.99) {
    double percent = fraction * 100.0;
    RCLCPP_ERROR(get_logger(), "Cartesian path for %s only covered %.0f%% of the line.", label.c_str(), percent);
    return false;
  }

  robot_trajectory::RobotTrajectory retimed(move_group_->getRobotModel(), move_group_->getName());
  moveit::core::RobotStatePtr current = move_group_->getCurrentState(10.0);
  retimed.setRobotTrajectoryMsg(*current, trajectory);
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  bool timed = totg.computeTimeStamps(retimed, test_speed_, test_speed_);
  if (!timed) {
    RCLCPP_ERROR(get_logger(), "Time parameterization failed: %s", label.c_str());
    return false;
  }
  retimed.getRobotTrajectoryMsg(trajectory);

  RCLCPP_INFO(get_logger(), "Sweeping: %s...", label.c_str());
  bool executed = (move_group_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!executed) {
    RCLCPP_ERROR(get_logger(), "Execution failed: %s", label.c_str());
    return false;
  }
  return true;
}

bool AxisTest::closerCandidate(const IkCandidate &a, const IkCandidate &b) {
  return a.distance < b.distance;
}

bool AxisTest::isDuplicate(const std::vector<double> &solution, const std::vector<IkCandidate> &candidates) {
  for (size_t i = 0; i < candidates.size(); i++) {
    double difference = 0.0;
    for (size_t j = 0; j < solution.size(); j++) {
      difference = difference + std::abs(solution[j] - candidates[i].joints[j]);
    }
    if (difference < 0.01) { return true; }
  }
  return false;
}

void AxisTest::wrapToNearest(std::vector<double> &solution, const std::vector<double> &current, const moveit::core::JointModelGroup *group) {
  const std::vector<const moveit::core::JointModel::Bounds *> &all_bounds = group->getActiveJointModelsBounds();
  for (size_t i = 0; i < solution.size(); i++) {
    const moveit::core::VariableBounds &bounds = (*all_bounds[i])[0];
    double best = solution[i];
    for (int k = -2; k <= 2; k++) {
      double shifted = solution[i] + k * 2.0 * M_PI;
      if (shifted < bounds.min_position_) { continue; }
      if (shifted > bounds.max_position_) { continue; }
      double shifted_distance = std::abs(shifted - current[i]);
      double best_distance = std::abs(best - current[i]);
      if (shifted_distance < best_distance) {
        best = shifted;
      }
    }
    solution[i] = best;
  }
}

geometry_msgs::msg::Quaternion AxisTest::pitchQuaternion(double pitch) {
  geometry_msgs::msg::Quaternion q;
  q.y = std::sin(pitch / 2.0);
  q.w = std::cos(pitch / 2.0);
  return q;
}

geometry_msgs::msg::Pose AxisTest::makePose(double x, double y, double z, const geometry_msgs::msg::Quaternion &q) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = q;
  return pose;
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
