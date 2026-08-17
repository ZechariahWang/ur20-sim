#include "utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

namespace utils {

// One IK solution and how far its joints are from the current joints.
struct IkCandidate {
  double distance;
  std::vector<double> joints;
};

static bool closerCandidate(const IkCandidate &a, const IkCandidate &b) {
  return a.distance < b.distance;
}

// True if this solution is nearly identical to one already collected.
static bool isDuplicate(const std::vector<double> &solution, const std::vector<IkCandidate> &candidates) {
  for (size_t i = 0; i < candidates.size(); i++) {
    double difference = 0.0;
    for (size_t j = 0; j < solution.size(); j++) {
      difference = difference + std::abs(solution[j] - candidates[i].joints[j]);
    }
    if (difference < 0.01) { return true; }
  }
  return false;
}

// Shift each joint by a multiple of 2*pi so it lands on the equivalent
// angle closest to its current value, staying inside the joint limits.
static void wrapToNearest(std::vector<double> &solution, const std::vector<double> &current, const moveit::core::JointModelGroup *group) {
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

geometry_msgs::msg::Quaternion pitchQuaternion(double pitch) {
  geometry_msgs::msg::Quaternion q;
  q.y = std::sin(pitch / 2.0);
  q.w = std::cos(pitch / 2.0);
  return q;
}

geometry_msgs::msg::Quaternion rollQuaternion(double roll) {
  geometry_msgs::msg::Quaternion q;
  q.x = std::sin(roll / 2.0);
  q.w = std::cos(roll / 2.0);
  return q;
}

geometry_msgs::msg::Pose makePose(double x, double y, double z, const geometry_msgs::msg::Quaternion &q) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = q;
  return pose;
}

bool moveToPose(moveit::planning_interface::MoveGroupInterface &move_group,
                const rclcpp::Logger &logger,
                const geometry_msgs::msg::Pose &pose, const std::string &label) {

  // Where the joints are right now.
  moveit::core::RobotStatePtr current = move_group.getCurrentState(10.0);
  const moveit::core::JointModelGroup *group = current->getJointModelGroup(move_group.getName());
  std::vector<double> current_joints;
  current->copyJointGroupPositions(group, current_joints);

  // Solve IK from several seeds and collect distinct solutions.
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
    RCLCPP_ERROR(logger, "No reachable IK solution: %s", label.c_str());
    return false;
  }

  // Try the closest solution first, fall back to the next closest if
  // planning rejects one.
  std::sort(candidates.begin(), candidates.end(), closerCandidate);
  for (size_t i = 0; i < candidates.size(); i++) {
    IkCandidate candidate = candidates[i];
    move_group.setJointValueTarget(candidate.joints);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!planned) {
      RCLCPP_WARN(logger, "Candidate %zu rejected for %s, trying next.", i, label.c_str());
      continue;
    }

    RCLCPP_INFO(logger, "Moving: %s (joint distance %.2f rad)...", label.c_str(), candidate.distance);
    bool executed = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!executed) {
      RCLCPP_ERROR(logger, "Execution failed: %s", label.c_str());
      return false;
    }
    return true;
  }

  RCLCPP_ERROR(logger, "Planning failed for every IK solution: %s", label.c_str());
  return false;
}

bool moveToJoints(moveit::planning_interface::MoveGroupInterface &move_group,
                  const rclcpp::Logger &logger,
                  const std::vector<double> &target, const std::string &label) {
  move_group.setJointValueTarget(target);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool planned = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!planned) {
    RCLCPP_ERROR(logger, "Planning failed: %s", label.c_str());
    return false;
  }
  RCLCPP_INFO(logger, "Moving: %s...", label.c_str());
  bool executed = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!executed) {
    RCLCPP_ERROR(logger, "Execution failed: %s", label.c_str());
    return false;
  }
  return true;
}

bool sweepTo(moveit::planning_interface::MoveGroupInterface &move_group,
             const rclcpp::Logger &logger, double eef_step,
             double velocity_scaling, double acceleration_scaling,
             const geometry_msgs::msg::Pose &pose, const std::string &label) {

  // Ask MoveIt for a straight-line path to the pose.
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(pose);
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, 0.0, trajectory);
  if (fraction < 0.99) {
    double percent = fraction * 100.0;
    RCLCPP_ERROR(logger, "Cartesian path for %s only covered %.0f%% of the line.", label.c_str(), percent);
    return false;
  }

  // The path comes back timed at full speed, slow it down to the
  // configured velocity and acceleration scaling.
  robot_trajectory::RobotTrajectory retimed(move_group.getRobotModel(), move_group.getName());
  moveit::core::RobotStatePtr current = move_group.getCurrentState(10.0);
  retimed.setRobotTrajectoryMsg(*current, trajectory);
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  bool timed = totg.computeTimeStamps(retimed, velocity_scaling, acceleration_scaling);
  if (!timed) {
    RCLCPP_ERROR(logger, "Time parameterization failed: %s", label.c_str());
    return false;
  }
  retimed.getRobotTrajectoryMsg(trajectory);

  RCLCPP_INFO(logger, "Sweeping: %s...", label.c_str());
  bool executed = (move_group.execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!executed) {
    RCLCPP_ERROR(logger, "Execution failed: %s", label.c_str());
    return false;
  }
  return true;
}

}
