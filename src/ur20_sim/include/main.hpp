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
    };

    // One IK solution and how far its joints are from the current joints.
    struct IkCandidate {
      double distance;
      std::vector<double> joints;
    };

    void loadParameters();
    bool setup();
    std::vector<Step> buildScript();
    bool checkAgainstRoom(const std::vector<Step> &script);
    bool doMovement();

    bool moveToPose(const geometry_msgs::msg::Pose &pose, const std::string &label);
    bool moveToJoints(const std::vector<double> &target, const std::string &label);
    bool sweepTo(const geometry_msgs::msg::Pose &pose, const std::string &label);

    static bool closerCandidate(const IkCandidate &a, const IkCandidate &b);
    static bool isDuplicate(const std::vector<double> &solution, const std::vector<IkCandidate> &candidates);
    static void wrapToNearest(std::vector<double> &solution, const std::vector<double> &current, const moveit::core::JointModelGroup *group);
    static geometry_msgs::msg::Quaternion pitchQuaternion(double pitch);
    static geometry_msgs::msg::Quaternion rollQuaternion(double roll);
    static geometry_msgs::msg::Pose makePose(double x, double y, double z, const geometry_msgs::msg::Quaternion &q);

    void stopSpinner();

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spinner_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    // Sweep geometry and speeds (main_sweep.yaml)
    std::string planning_group_;
    double velocity_scaling_{0.05};
    double acceleration_scaling_{0.1};
    double sweep_x_{1.0};
    double sweep_z_{0.35};
    double sweep_y_start_{0.9};
    double sweep_y_end_{-0.9};
    double eef_step_{0.01};
    std::vector<double> park_joints_;

    // Room bounds (room.yaml) and how close the TCP may get to them.
    double floor_z_{-0.80};
    double ceiling_z_{1.80};
    double x_min_{-1.12};
    double x_max_{2.50};
    double y_min_{-1.65};
    double y_max_{1.65};
    double room_margin_{0.15};
};

#endif
