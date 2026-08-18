#include "tcp_pose_publisher.hpp"

#include <chrono>
#include <memory>

TcpPosePublisher::TcpPosePublisher()
    : Node("tcp_pose_publisher"),
      buffer_(get_clock()),
      listener_(buffer_) {
  publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/tcp_pose_broadcaster/pose", 10);
  timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { publishPose(); });
}

void TcpPosePublisher::publishPose() {
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
  publisher_->publish(pose);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TcpPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
