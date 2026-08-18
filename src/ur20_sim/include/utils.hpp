#ifndef UR20_SIM_UTILS_HPP
#define UR20_SIM_UTILS_HPP

#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

// Common motion helpers shared by the sweep routine and the calibration
// tests. All poses are in the ROS base frame (open workspace +x, wall
// behind the robot -x).
namespace utils {

// Quaternion for a pure pitch (rotation about the base Y axis).
// pi = TCP pointing straight down, pi/2 = horizontal pointing +x.
geometry_msgs::msg::Quaternion pitchQuaternion(double pitch);

// Quaternion for a pure roll (rotation about the base X axis).
// Tilts the view within the vertical plane of a y-axis sweep line.
geometry_msgs::msg::Quaternion rollQuaternion(double roll);

geometry_msgs::msg::Pose makePose(double x, double y, double z, const geometry_msgs::msg::Quaternion &q);

// TCP orientation the arm would have at the given joint configuration
// (forward kinematics, no motion).
geometry_msgs::msg::Quaternion orientationFromJoints(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const std::vector<double> &joints);

// Full TCP pose (position + orientation) the arm would have at the given
// joint configuration (forward kinematics, no motion).
geometry_msgs::msg::Pose poseFromJoints(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const std::vector<double> &joints);

// The target joints shifted by multiples of 2*pi to the equivalents
// nearest the CURRENT joints (within limits). Same physical pose; useful
// for measuring how far a move really is before committing to it.
std::vector<double> nearestJointTarget(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const std::vector<double> &target);

// The camera sticks out of the flange along tool z. Given where the
// camera TIP should be, return where the flange must go: the same pose
// pulled back by camera_length along the tool z axis.
geometry_msgs::msg::Pose flangePoseFromCameraPose(
    const geometry_msgs::msg::Pose &camera_pose, double camera_length);

// Free repositioning move to a pose. Solves IK from several seeds, wraps
// each solution to the 2*pi-equivalent nearest the current joints, and
// plans to the closest solution first, falling back to the next closest
// if planning rejects one (for example a room collision).
bool moveToPose(moveit::planning_interface::MoveGroupInterface &move_group,
                const rclcpp::Logger &logger,
                const geometry_msgs::msg::Pose &pose, const std::string &label);

// Same, but solves IK seeded from a demonstrated joint configuration so
// the arm lands in that specific configuration family (for example the
// branch the operator jogged to). Falls back to moveToPose if the seeded
// solve fails.
bool moveToPoseSeeded(moveit::planning_interface::MoveGroupInterface &move_group,
                      const rclcpp::Logger &logger,
                      const geometry_msgs::msg::Pose &pose,
                      const std::vector<double> &seed_joints, const std::string &label);

// Joint-space move to an exact joint configuration.
bool moveToJoints(moveit::planning_interface::MoveGroupInterface &move_group,
                  const rclcpp::Logger &logger,
                  const std::vector<double> &target, const std::string &label);

// Same, but first shifts each target joint by multiples of 2*pi to the
// equivalent angle nearest the current joints (within limits): the same
// physical pose, reached the short way instead of unwinding a full turn.
bool moveToNearestJoints(moveit::planning_interface::MoveGroupInterface &move_group,
                         const rclcpp::Logger &logger,
                         const std::vector<double> &target, const std::string &label);

// Straight Cartesian line from the current TCP pose to the given pose,
// retimed to the given velocity/acceleration scaling.
bool sweepTo(moveit::planning_interface::MoveGroupInterface &move_group,
             const rclcpp::Logger &logger, double eef_step,
             double velocity_scaling, double acceleration_scaling,
             const geometry_msgs::msg::Pose &pose, const std::string &label);

}

#endif
