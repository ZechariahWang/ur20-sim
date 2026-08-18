#ifndef UR20_SIM_TCP_POSE_PUBLISHER_HPP
#define UR20_SIM_TCP_POSE_PUBLISHER_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Fake hardware only.
class TcpPosePublisher : public rclcpp::Node {

  public:

    TcpPosePublisher();

  private:

    void publishPose();

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
