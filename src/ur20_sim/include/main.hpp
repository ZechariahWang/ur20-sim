#ifndef UR20_SIM_MAIN_HPP
#define UR20_SIM_MAIN_HPP

#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

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
    std::vector<Step> buildScript(const geometry_msgs::msg::Quaternion &top_orientation, const geometry_msgs::msg::Quaternion &side_orientation);
    bool checkAgainstRoom(const std::vector<Step> &script);
    bool doMovement();
    void stopSpinner();

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spinner_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

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
    std::vector<double> park_joints_;
    // Joint pose whose TCP orientation is used for the second (side) pass.
    std::vector<double> side_view_joints_;
    // Joint pose for the final oblique view after both passes.
    std::vector<double> oblique_view_joints_;

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
