#include "sim_state_publisher.hpp"

#include <chrono>
#include <memory>

SimStatePublisher::SimStatePublisher()
    : Node("sim_state_publisher"),
      buffer_(get_clock()),
      listener_(buffer_) {
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/tcp_pose_broadcaster/pose", 10);
  timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { publishPose(); });

  // Same QoS as the real driver: latched, so the webapp gets the value
  // right after subscribing.
  rclcpp::QoS latched(1);
  latched.transient_local();
  robot_mode_pub_ = create_publisher<ur_dashboard_msgs::msg::RobotMode>(
      "/io_and_status_controller/robot_mode", latched);
  safety_mode_pub_ = create_publisher<ur_dashboard_msgs::msg::SafetyMode>(
      "/io_and_status_controller/safety_mode", latched);
  program_running_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/io_and_status_controller/robot_program_running", latched);

  ur_dashboard_msgs::msg::RobotMode robot_mode;
  robot_mode.mode = ur_dashboard_msgs::msg::RobotMode::RUNNING;
  robot_mode_pub_->publish(robot_mode);

  ur_dashboard_msgs::msg::SafetyMode safety_mode;
  safety_mode.mode = ur_dashboard_msgs::msg::SafetyMode::NORMAL;
  safety_mode_pub_->publish(safety_mode);

  std_msgs::msg::Bool program_running;
  program_running.data = true;
  program_running_pub_->publish(program_running);
}

void SimStatePublisher::publishPose() {
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = buffer_.lookupTransform("base", "tool0", tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    // tf is not ready yet, try again on the next tick.
    return;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = transform.header.stamp;
  pose.header.frame_id = "base";
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;
  pose_pub_->publish(pose);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
