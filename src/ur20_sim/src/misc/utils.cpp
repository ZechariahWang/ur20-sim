#include "utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include <Eigen/Geometry>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

namespace utils {

// One IK solution, how far its joints are from the current joints, and
// its selection score (distance plus a penalty for wound-up joints).
struct IkCandidate {
  double distance;
  double score;
  std::vector<double> joints;
};

static bool betterCandidate(const IkCandidate &a, const IkCandidate &b) {
  return a.score < b.score;
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

// Shift each joint by multiples of 2*pi into [-pi, pi] where the limits
// allow: the same physical pose in its least wound-up form.
static std::vector<double> unwoundVariant(const std::vector<double> &solution, const moveit::core::JointModelGroup *group) {
  const std::vector<const moveit::core::JointModel::Bounds *> &all_bounds = group->getActiveJointModelsBounds();
  std::vector<double> unwound = solution;
  for (size_t i = 0; i < unwound.size(); i++) {
    const moveit::core::VariableBounds &bounds = (*all_bounds[i])[0];
    while (unwound[i] > M_PI && unwound[i] - 2.0 * M_PI >= bounds.min_position_) {
      unwound[i] = unwound[i] - 2.0 * M_PI;
    }
    while (unwound[i] < -M_PI && unwound[i] + 2.0 * M_PI <= bounds.max_position_) {
      unwound[i] = unwound[i] + 2.0 * M_PI;
    }
  }
  return unwound;
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

geometry_msgs::msg::Quaternion orientationFromJoints(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const std::vector<double> &joints) {
  moveit::core::RobotStatePtr state = move_group.getCurrentState(10.0);
  const moveit::core::JointModelGroup *group = state->getJointModelGroup(move_group.getName());
  state->setJointGroupPositions(group, joints);
  state->update();
  const Eigen::Isometry3d &tcp = state->getGlobalLinkTransform(move_group.getEndEffectorLink());
  Eigen::Quaterniond eigen_q(tcp.rotation());
  eigen_q.normalize();

  geometry_msgs::msg::Quaternion q;
  q.x = eigen_q.x();
  q.y = eigen_q.y();
  q.z = eigen_q.z();
  q.w = eigen_q.w();
  return q;
}

geometry_msgs::msg::Pose poseFromJoints(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const std::vector<double> &joints) {
  moveit::core::RobotStatePtr state = move_group.getCurrentState(10.0);
  const moveit::core::JointModelGroup *group = state->getJointModelGroup(move_group.getName());
  state->setJointGroupPositions(group, joints);
  state->update();
  const Eigen::Isometry3d &tcp = state->getGlobalLinkTransform(move_group.getEndEffectorLink());
  Eigen::Quaterniond eigen_q(tcp.rotation());
  eigen_q.normalize();

  geometry_msgs::msg::Pose pose;
  pose.position.x = tcp.translation().x();
  pose.position.y = tcp.translation().y();
  pose.position.z = tcp.translation().z();
  pose.orientation.x = eigen_q.x();
  pose.orientation.y = eigen_q.y();
  pose.orientation.z = eigen_q.z();
  pose.orientation.w = eigen_q.w();
  return pose;
}

std::vector<double> nearestJointTarget(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const std::vector<double> &target) {
  moveit::core::RobotStatePtr current = move_group.getCurrentState(10.0);
  const moveit::core::JointModelGroup *group = current->getJointModelGroup(move_group.getName());
  std::vector<double> current_joints;
  current->copyJointGroupPositions(group, current_joints);

  std::vector<double> wrapped = target;
  wrapToNearest(wrapped, current_joints, group);
  return wrapped;
}

geometry_msgs::msg::Pose flangePoseFromCameraPose(
    const geometry_msgs::msg::Pose &camera_pose, double camera_length) {
  Eigen::Quaterniond q(camera_pose.orientation.w, camera_pose.orientation.x,
                       camera_pose.orientation.y, camera_pose.orientation.z);
  Eigen::Vector3d camera_axis = q * Eigen::Vector3d(0.0, 0.0, camera_length);

  geometry_msgs::msg::Pose flange_pose = camera_pose;
  flange_pose.position.x = camera_pose.position.x - camera_axis.x();
  flange_pose.position.y = camera_pose.position.y - camera_axis.y();
  flange_pose.position.z = camera_pose.position.z - camera_axis.z();
  return flange_pose;
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

    // Penalize wound-up configurations (joints beyond +-180 deg). They may
    // be close to the current pose now, but every later move out of them
    // forces a huge unwinding rotation.
    double winding = 0.0;
    for (size_t i = 0; i < solution.size(); i++) {
      double beyond = std::abs(solution[i]) - M_PI;
      if (beyond > 0.0) { winding = winding + beyond; }
    }

    if (!isDuplicate(solution, candidates)) {
      candidates.push_back({distance, distance + winding, solution});
    }

    std::vector<double> unwound = unwoundVariant(solution, group);
    if (!isDuplicate(unwound, candidates)) {
      double unwound_distance = 0.0;
      for (size_t i = 0; i < unwound.size(); i++) {
        unwound_distance = unwound_distance + std::abs(unwound[i] - current_joints[i]);
      }
      double unwound_winding = 0.0;
      for (size_t i = 0; i < unwound.size(); i++) {
        double beyond = std::abs(unwound[i]) - M_PI;
        if (beyond > 0.0) { unwound_winding = unwound_winding + beyond; }
      }
      candidates.push_back({unwound_distance, unwound_distance + unwound_winding, unwound});
    }
  }
  if (candidates.empty()) {
    RCLCPP_ERROR(logger, "No reachable IK solution: %s", label.c_str());
    return false;
  }

  // Try the closest solution first, fall back to the next closest if
  // planning rejects one.
  std::sort(candidates.begin(), candidates.end(), betterCandidate);
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
    RCLCPP_INFO(logger, "  chosen config: %.2f %.2f %.2f %.2f %.2f %.2f",
                candidate.joints[0], candidate.joints[1], candidate.joints[2],
                candidate.joints[3], candidate.joints[4], candidate.joints[5]);
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

bool moveToPoseSeeded(moveit::planning_interface::MoveGroupInterface &move_group,
                      const rclcpp::Logger &logger,
                      const geometry_msgs::msg::Pose &pose,
                      const std::vector<double> &seed_joints, const std::string &label) {
  moveit::core::RobotStatePtr current = move_group.getCurrentState(10.0);
  const moveit::core::JointModelGroup *group = current->getJointModelGroup(move_group.getName());

  moveit::core::RobotState state(*current);
  state.setJointGroupPositions(group, seed_joints);
  bool found = state.setFromIK(group, pose, 0.1);
  if (!found) {
    RCLCPP_WARN(logger, "Seeded IK failed for %s, falling back to unseeded.", label.c_str());
    return moveToPose(move_group, logger, pose, label);
  }

  std::vector<double> solution;
  state.copyJointGroupPositions(group, solution);
  // Keep the solution in the seed's winding, not the current pose's.
  wrapToNearest(solution, seed_joints, group);

  move_group.setJointValueTarget(solution);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool planned = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!planned) {
    RCLCPP_WARN(logger, "Seeded plan failed for %s, falling back to unseeded.", label.c_str());
    return moveToPose(move_group, logger, pose, label);
  }

  RCLCPP_INFO(logger, "Moving: %s (seeded)...", label.c_str());
  RCLCPP_INFO(logger, "  chosen config: %.2f %.2f %.2f %.2f %.2f %.2f",
              solution[0], solution[1], solution[2], solution[3], solution[4], solution[5]);
  bool executed = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!executed) {
    RCLCPP_ERROR(logger, "Execution failed: %s", label.c_str());
    return false;
  }
  return true;
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

bool moveToNearestJoints(moveit::planning_interface::MoveGroupInterface &move_group, const rclcpp::Logger &logger, const std::vector<double> &target, const std::string &label) {
  moveit::core::RobotStatePtr current = move_group.getCurrentState(10.0);
  const moveit::core::JointModelGroup *group = current->getJointModelGroup(move_group.getName());
  std::vector<double> current_joints;
  current->copyJointGroupPositions(group, current_joints);

  std::vector<double> wrapped = target;
  wrapToNearest(wrapped, current_joints, group);
  for (size_t i = 0; i < wrapped.size(); i++) {
    RCLCPP_INFO(logger, "  joint %zu: current %.2f -> target %.2f (delta %.2f rad)",
                i, current_joints[i], wrapped[i], wrapped[i] - current_joints[i]);
  }
  return moveToJoints(move_group, logger, wrapped, label);
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

  moveit::core::RobotStatePtr after = move_group.getCurrentState(10.0);
  std::vector<double> joints_after;
  after->copyJointGroupPositions(after->getJointModelGroup(move_group.getName()), joints_after);
  RCLCPP_INFO(logger, "  joints after sweep: %.2f %.2f %.2f %.2f %.2f %.2f",
              joints_after[0], joints_after[1], joints_after[2],
              joints_after[3], joints_after[4], joints_after[5]);
  return true;
}

}
