#ifndef UR20_SIM_CALIBRATION_TESTS_SWEEP_MOVER_HPP
#define UR20_SIM_CALIBRATION_TESTS_SWEEP_MOVER_HPP

#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

// Calibration version of the scanning routine: park, top sweep, side
// sweep, oblique view. No room pre-check; use main_sweep for production.
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

    void loadParameters();
    bool setup();
    bool doMovement();
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
