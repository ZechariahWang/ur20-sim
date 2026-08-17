#ifndef UR20_SIM_CALIBRATION_TESTS_AXIS_TEST_HPP
#define UR20_SIM_CALIBRATION_TESTS_AXIS_TEST_HPP

#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

// Calibration test: from the park pose, move to a center point, then
// sweep left to right, then up and down, then rotate the TCP in place.
class AxisTest : public rclcpp::Node {

  public:

    AxisTest();
    ~AxisTest() override;

    bool run();

  private:

    void loadParameters();
    bool setup();
    bool doMovement();
    void stopSpinner();

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spinner_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    std::string planning_group_;
    double test_speed_{0.03};
    double center_x_{1.0};
    double center_y_{0.0};
    double center_z_{0.35};
    double y_travel_{0.4};
    double z_travel_{0.25};
    double rotation_rad_{1.57};
    double eef_step_{0.01};
    std::vector<double> park_joints_;
};

#endif
