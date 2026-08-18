#ifndef UR20_SIM_COMMAND_SERVER_HPP
#define UR20_SIM_COMMAND_SERVER_HPP

#include <string>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

// Lets the webapp control the robot over rosbridge. Listens for simple
// text commands on /webapp/command and reports on /webapp/status.
// Commands:
//   "sweep" - run the full main sweep routine
//   "park"  - move to the park pose only
//   "stop"  - abort the running routine and cancel the active trajectory
class CommandServer : public rclcpp::Node {

  public:

    CommandServer();

  private:

    void onCommand(const std_msgs::msg::String &msg);
    void startRoutine(const std::string &command);
    void stopRoutine();
    void checkChild();
    void publishStatus(const std::string &status);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_client_;

    // Process id of the running routine, -1 when idle.
    int child_pid_{-1};
    bool stopping_{false};
    std::string running_command_;
};

#endif
