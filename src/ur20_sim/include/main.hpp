#ifndef UR20_SIM_MAIN_HPP
#define UR20_SIM_MAIN_HPP

#include <atomic>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

// The production sweep routine: park, top sweep, side sweep, oblique view.
// Before moving it checks every scripted pose against the measured room
// bounds (loaded from room.yaml) and refuses to start if any pose is too
// close to the floor, ceiling, or walls.
class MainSweep : public rclcpp::Node {

  public:

    MainSweep();
    ~MainSweep() override;

    bool run();

  private:

    // One entry in the movement script.
    // MOVE = free repositioning move (shortest joint path).
    // SWEEP = straight Cartesian line from the current TCP pose to this pose.
    struct Step {
      enum Type { MOVE, SWEEP } type;
      geometry_msgs::msg::Pose pose;
      std::string label;
      // Optional: IK seed so this move lands in a demonstrated
      // configuration family. Empty = pick nearest to current.
      std::vector<double> seed;
    };

    void loadParameters();
    bool setup();
    // Picks the script for the configured board type.
    std::vector<Step> buildScript(const geometry_msgs::msg::Quaternion &top_orientation, const geometry_msgs::msg::Quaternion &side_orientation);
    // One script per board type. Currently identical; edit independently
    // when the boards need different paths.
    std::vector<Step> buildLargeBoardScript(const geometry_msgs::msg::Quaternion &top_orientation, const geometry_msgs::msg::Quaternion &side_orientation);
    std::vector<Step> buildSmallBoardScript(const geometry_msgs::msg::Quaternion &top_orientation, const geometry_msgs::msg::Quaternion &side_orientation);
    // "angled" mode: one left-to-right sweep that holds the height and
    // orientation of the jogged angled pose (angled_view_joints) the
    // whole way, then straight back to park.
    std::vector<Step> buildAngledScript();
    bool checkAgainstRoom(const std::vector<Step> &script);
    bool doMovement();
    void pauseAtPose(const std::string &label, double seconds);
    // Runs one motion, holding and retrying while the webapp pause flag
    // is up. On resume the motion replans from wherever the arm stopped.
    bool withPauseRetry(const std::function<bool()> &motion, const std::string &label);
    void waitWhilePaused();
    void stopSpinner();

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spinner_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
    // True while the webapp holds the routine (latched /webapp/paused).
    std::atomic<bool> paused_{false};

    // Which board is being scanned: "large" or "small". Selects the
    // sweep script in buildScript().
    std::string board_type_{"large"};

    // Sweep geometry and speeds (main_sweep.yaml)
    std::string planning_group_;
    double velocity_scaling_{0.05};
    double acceleration_scaling_{0.1};
    double sweep_x_{1.0};
    // Pass heights above the real floor (m), not the robot base.
    double top_pass_height_{0.30};
    double side_pass_height_{0.06};
    double sweep_y_start_{0.9};
    double sweep_y_end_{-0.9};
    double eef_step_{0.01};
    // How far the camera tip sticks out of the flange along tool z (m).
    double camera_length_{0.22};
    // How long to hold still at each reached pose (s).
    double pose_pause_seconds_{2.0};
    // Longer hold at poses that start or end a sweep (large board only).
    double sweep_pause_seconds_{5.0};
    // Shift of the oblique view relative to the jogged pose (m, base frame).
    double oblique_shift_x_{0.15};
    double oblique_shift_y_{0.15};
    double oblique_shift_z_{0.0};
    std::vector<double> park_joints_;
    // If true, only move to park and exit (webapp "park" command).
    bool park_only_{false};
    // Joint pose whose TCP orientation is used for the second (side) pass.
    std::vector<double> side_view_joints_;
    // Joint pose for the final oblique view after both passes.
    std::vector<double> oblique_view_joints_;
    // Jogged joint pose whose FK height and orientation the "angled"
    // sweep holds for the whole pass.
    std::vector<double> angled_view_joints_;

    // Room bounds (room.yaml) and how close the TCP may get to them.
    double floor_z_{-0.81};
    double ceiling_z_{1.79};
    double x_min_{-1.12};
    double x_max_{2.50};
    double y_min_{-1.65};
    double y_max_{1.65};
    double room_margin_{0.15};
    // Separate, smaller margin for the floor so the side pass may fly low.
    double floor_margin_{0.03};
};

#endif
