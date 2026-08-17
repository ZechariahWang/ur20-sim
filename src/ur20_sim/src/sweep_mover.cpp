#include "sweep_mover.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

SweepMover::SweepMover()
    : Node("sweep_mover",
          rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {}

SweepMover::~SweepMover() {
  stopSpinner();
}

bool SweepMover::run() {
  loadParameters();
  if (!setup()) { return false; }
  bool ok = doMovement();
  if (ok) {
    RCLCPP_INFO(get_logger(), "Sweep complete.");
  } else {
    RCLCPP_INFO(get_logger(), "Sweep finished with errors.");
  }
  return ok;
}

void SweepMover::loadParameters() {
  planning_group_ = get_parameter_or<std::string>("planning_group", "ur_manipulator");
  velocity_scaling_ = get_parameter_or<double>("velocity_scaling", 0.3);
  acceleration_scaling_ = get_parameter_or<double>("acceleration_scaling", 0.3);
  sweep_x_ = get_parameter_or<double>("sweep_x", 1.0);
  sweep_z_ = get_parameter_or<double>("sweep_z", 0.35);
  sweep_y_start_ = get_parameter_or<double>("sweep_y_start", 0.9);
  sweep_y_end_ = get_parameter_or<double>("sweep_y_end", -0.9);
  eef_step_ = get_parameter_or<double>("eef_step", 0.01);
  park_joints_ = get_parameter_or<std::vector<double>>("park_joints", {});
}

bool SweepMover::setup() {

  // MoveGroupInterface needs the node spinning in the background for its action clients and TF lookups.
  executor_.add_node(shared_from_this());
  spinner_ = std::thread([this]() { executor_.spin(); });

  move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group_);
  move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);

  std::string frame = move_group_->getPlanningFrame();
  std::string tcp_link = move_group_->getEndEffectorLink();
  RCLCPP_INFO(get_logger(), "Sweeping in frame '%s' with TCP link '%s'.", frame.c_str(), tcp_link.c_str());

  // Give DDS discovery a moment to connect, otherwise the first planning request can be lost.
  rclcpp::sleep_for(std::chrono::seconds(1));
  return true;
}

bool SweepMover::doMovement() {

  // First move to the parked pose (joint-space, exact) so every run
  // starts the routine from the same place no matter where the arm was.
  if (park_joints_.size() == 6) {
    if (!moveToJoints(park_joints_, "move to park position")) { return false; }
  }

  // TCP orientations. In ROS base coordinates the open workspace is on
  // +x and the wall is behind the robot on -x (the pendant base frame
  // is rotated 180 deg about z, so pendant x is the opposite sign).
  geometry_msgs::msg::Quaternion down = pitchQuaternion(M_PI);
  geometry_msgs::msg::Quaternion side = pitchQuaternion(M_PI / 2.0);

  // Negative roll tilts left (toward the start), positive tilts right.
  // -2.0 * M_PI / 3.0 (-120 deg) = shallower, 30 down / 60 left
  // -3.0 * M_PI / 4.0 (-135 deg) = even split, down and left
  // +3.0 * M_PI / 4.0 (+135 deg) = current: even split, down and right
  // -5.0 * M_PI / 6.0 (-150 deg) = steeper, 60 down / 30 left
  geometry_msgs::msg::Quaternion oblique = rollQuaternion(3.0 * M_PI / 4.0);

  double x = sweep_x_;
  double z = sweep_z_;
  double start = sweep_y_start_;
  double end = sweep_y_end_;

  std::vector<Step> script;
  script.push_back({Step::MOVE,  makePose(x, start, z, down),          "sweep start"});
  script.push_back({Step::SWEEP, makePose(x, end,   z, down),          "top sweep"});
  script.push_back({Step::MOVE,  makePose(x-0.1, start, z, side),          "return to start (side view)"});
  script.push_back({Step::SWEEP, makePose(x-0.1, end,   z, side),          "side sweep"});
  script.push_back({Step::MOVE,  makePose(x-0.2, end,   z + 0.3, oblique), "oblique view"});

  for (size_t i = 0; i < script.size(); i++) {
    Step step = script[i];
    bool ok = false;
    if (step.type == Step::MOVE) {
      ok = moveToPose(step.pose, step.label);
    } else {
      ok = sweepTo(step.pose, step.label);
    }
    if (!ok) { return false; }
  }
  return true;
}

bool SweepMover::moveToPose(const geometry_msgs::msg::Pose &pose, const std::string &label) {

  // Where the joints are right now.
  moveit::core::RobotStatePtr current = move_group_->getCurrentState(10.0);
  const moveit::core::JointModelGroup *group = current->getJointModelGroup(planning_group_);
  std::vector<double> current_joints;
  current->copyJointGroupPositions(group, current_joints);

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

  // Try the closest solution first
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

bool SweepMover::moveToJoints(const std::vector<double> &target, const std::string &label) {
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

bool SweepMover::sweepTo(const geometry_msgs::msg::Pose &pose, const std::string &label) {

  // Ask MoveIt for a straight-line path to the pose.
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(pose);
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group_->computeCartesianPath(waypoints, eef_step_, 0.0, trajectory);
  if (fraction < 0.99) {
    double percent = fraction * 100.0;
    RCLCPP_ERROR(get_logger(), "Cartesian path for %s only covered %.0f%% of the line.", label.c_str(), percent);
    return false;
  }

  // slow it down to the configured velocity and acceleration scaling.
  robot_trajectory::RobotTrajectory retimed(move_group_->getRobotModel(), move_group_->getName());
  moveit::core::RobotStatePtr current = move_group_->getCurrentState(10.0);
  retimed.setRobotTrajectoryMsg(*current, trajectory);
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  bool timed = totg.computeTimeStamps(retimed, velocity_scaling_, acceleration_scaling_);
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

bool SweepMover::closerCandidate(const IkCandidate &a, const IkCandidate &b) {
  return a.distance < b.distance;
}

// True if this solution is nearly identical to one already collected.
bool SweepMover::isDuplicate(const std::vector<double> &solution, const std::vector<IkCandidate> &candidates) {
  for (size_t i = 0; i < candidates.size(); i++) {
    double difference = 0.0;
    for (size_t j = 0; j < solution.size(); j++) {
      difference = difference + std::abs(solution[j] - candidates[i].joints[j]);
    }
    if (difference < 0.01) { return true; }
  }
  return false;
}

// Shift each joint by a multiple of 2*pi
void SweepMover::wrapToNearest(std::vector<double> &solution, const std::vector<double> &current, const moveit::core::JointModelGroup *group) {
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

// Quaternion for a pure pitch (rotation about the base Y axis).
geometry_msgs::msg::Quaternion SweepMover::pitchQuaternion(double pitch) {
  geometry_msgs::msg::Quaternion q;
  q.y = std::sin(pitch / 2.0);
  q.w = std::cos(pitch / 2.0);
  return q;
}

// Quaternion for a pure roll (rotation about the base X axis).
geometry_msgs::msg::Quaternion SweepMover::rollQuaternion(double roll) {
  geometry_msgs::msg::Quaternion q;
  q.x = std::sin(roll / 2.0);
  q.w = std::cos(roll / 2.0);
  return q;
}

geometry_msgs::msg::Pose SweepMover::makePose(double x, double y, double z, const geometry_msgs::msg::Quaternion &q) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = q;
  return pose;
}

void SweepMover::stopSpinner() {
  if (spinner_.joinable()) {
    executor_.cancel();
    spinner_.join();
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SweepMover>();
  bool ok = node->run();
  rclcpp::shutdown();
  if (ok) { return 0; }
  return 1;
}
