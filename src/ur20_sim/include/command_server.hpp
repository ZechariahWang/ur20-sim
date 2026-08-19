#ifndef UR20_SIM_COMMAND_SERVER_HPP
#define UR20_SIM_COMMAND_SERVER_HPP

#include <string>

#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

// Commands:
//   "sweep"  - run the large board sweep routine
//   "small"  - run the small board routine (stationary views)
//   "park"   - move to the park pose only
//   "rest"   - fold the arm into the end of day rest pose
//   "pause"  - halt motion but keep the sequence loaded (status "paused:")
//   "resume" - continue the paused sequence from where it stopped
//   "stop"   - abort the running routine and cancel the active trajectory
// Pause works through the latched /webapp/paused flag: the routine nodes
// hold while it is true and replan from the current arm position on resume.
class CommandServer : public rclcpp::Node {

  public:

    CommandServer();
    ~CommandServer() override;

  private:

    void onCommand(const std_msgs::msg::String &msg);
    void startRoutine(const std::string &command);
    void stopRoutine();
    void pauseRoutine();
    void resumeRoutine();
    void checkChild();
    void checkExternalMotion();
    void publishStatus(const std::string &status);
    void publishPaused(bool paused);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr paused_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_client_;

    // Process id of the running routine, -1 when idle.
    int child_pid_{-1};
    bool stopping_{false};
    bool routine_paused_{false};
    std::string running_command_;

    // Motion the webapp did not start (for example a sweep launched from
    // a terminal) still shows up in the status as "running: external".
    std::vector<double> latest_positions_;
    std::vector<double> last_checked_positions_;
    bool external_moving_{false};
};

#endif
