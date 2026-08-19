#ifndef UR20_SIM_SIM_STATE_PUBLISHER_HPP
#define UR20_SIM_SIM_STATE_PUBLISHER_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ur_dashboard_msgs/msg/robot_mode.hpp>
#include <ur_dashboard_msgs/msg/safety_mode.hpp>

// Fake hardware only. On the real robot the driver publishes the TCP
// pose, robot mode, safety mode, and program state itself; with fake
// hardware those topics exist but stay silent (the webapp shows null).
// This node fills them with sim stand-ins so the webapp works
// identically in both modes:
//   /tcp_pose_broadcaster/pose               from tf (base -> tool0)
//   /io_and_status_controller/robot_mode     RUNNING, latched
//   /io_and_status_controller/safety_mode    NORMAL, latched
//   /io_and_status_controller/robot_program_running  true, latched
class SimStatePublisher : public rclcpp::Node {

  public:

    SimStatePublisher();

  private:

    void publishPose();

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<ur_dashboard_msgs::msg::RobotMode>::SharedPtr robot_mode_pub_;
    rclcpp::Publisher<ur_dashboard_msgs::msg::SafetyMode>::SharedPtr safety_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr program_running_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
