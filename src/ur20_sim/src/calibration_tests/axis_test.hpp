#ifndef UR20_SIM_CALIBRATION_TESTS_AXIS_TEST_HPP
#define UR20_SIM_CALIBRATION_TESTS_AXIS_TEST_HPP

#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
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
    static geometry_msgs::msg::Pose makePose(double x, double y, double z, const geometry_msgs::msg::Quaternion &q);

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
