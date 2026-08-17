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

// Free repositioning move to a pose. Solves IK from several seeds, wraps
// each solution to the 2*pi-equivalent nearest the current joints, and
// plans to the closest solution first, falling back to the next closest
// if planning rejects one (for example a room collision).
bool moveToPose(moveit::planning_interface::MoveGroupInterface &move_group,
                const rclcpp::Logger &logger,
                const geometry_msgs::msg::Pose &pose, const std::string &label);

// Joint-space move to an exact joint configuration.
bool moveToJoints(moveit::planning_interface::MoveGroupInterface &move_group,
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
