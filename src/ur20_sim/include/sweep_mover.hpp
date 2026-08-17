#ifndef UR20_SIM_SWEEP_MOVER_HPP
#define UR20_SIM_SWEEP_MOVER_HPP

#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

// Runs the scanning routine: park, top sweep, side sweep, oblique view.
class SweepMover : public rclcpp::Node {

  public:

    SweepMover();
    ~SweepMover() override;

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

    std::string planning_group_;
    double velocity_scaling_{0.3};
    double acceleration_scaling_{0.2};
    double sweep_x_{1.0};
    double sweep_z_{0.35};
    double sweep_y_start_{0.9};
    double sweep_y_end_{-0.9};
    double eef_step_{0.01};
    std::vector<double> park_joints_;
};

#endif
