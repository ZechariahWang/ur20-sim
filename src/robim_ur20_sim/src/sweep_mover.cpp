// Sweeps the UR20 TCP in a straight line across its workspace in three views:
//   1. along the line with the TCP pointing straight down at the ground,
//   2. back along the same line with the TCP pitched 90 deg, pointing away
//      from the robot,
//   3. then a final move to an oblique (45 deg) view at the line's midpoint.
// Line geometry and speeds are set in config/sweep.yaml.

#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <rclcpp/rclcpp.hpp>

class SweepMover : public rclcpp::Node
{
public:
  SweepMover()
  : Node(
      "sweep_mover",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
  }

  ~SweepMover() override
  {
    stopSpinner();
  }

  bool run()
  {
    executor_.add_node(shared_from_this());
    spinner_ = std::thread([this]() { executor_.spin(); });

    const auto planning_group =
      get_parameter_or<std::string>("planning_group", "ur_manipulator");
    velocity_scaling_ = get_parameter_or<double>("velocity_scaling", 0.3);
    acceleration_scaling_ = get_parameter_or<double>("acceleration_scaling", 0.3);
    eef_step_ = get_parameter_or<double>("eef_step", 0.01);

    // The sweep line runs parallel to the base Y axis at fixed x and z.
    const double x = get_parameter_or<double>("sweep_x", 0.7);
    const double z = get_parameter_or<double>("sweep_z", 0.4);
    const double y_min = get_parameter_or<double>("sweep_y_min", -0.9);
    const double y_max = get_parameter_or<double>("sweep_y_max", 0.9);
    const double y_mid = 0.5 * (y_min + y_max);

    moveit::planning_interface::MoveGroupInterface move_group(
      shared_from_this(), planning_group);
    move_group.setMaxVelocityScalingFactor(velocity_scaling_);
    move_group.setMaxAccelerationScalingFactor(acceleration_scaling_);

    RCLCPP_INFO(
      get_logger(), "Sweeping in frame '%s' with TCP link '%s'.",
      move_group.getPlanningFrame().c_str(), move_group.getEndEffectorLink().c_str());

    // Give DDS discovery a moment to match the action client, otherwise the
    // first goal's response can be lost and plan() blocks forever.
    rclcpp::sleep_for(std::chrono::seconds(1));

    // All three views are pitches about the base Y axis:
    // 180 deg = straight down, 90 deg = horizontal away from the robot.
    const auto down = pitchQuaternion(M_PI);
    const auto side = pitchQuaternion(M_PI / 2.0);
    const auto oblique = pitchQuaternion(3.0 * M_PI / 4.0);

    const bool ok =
      moveToPose(move_group, makePose(x, y_min, z, down), "sweep start (down view)") &&
      sweepTo(move_group, makePose(x, y_max, z, down), "sweep 1 (down view)") &&
      moveToPose(move_group, makePose(x, y_max, z, side), "reorient to side view") &&
      sweepTo(move_group, makePose(x, y_min, z, side), "sweep 2 (side view)") &&
      moveToPose(move_group, makePose(x, y_mid, z, oblique), "final oblique view");

    RCLCPP_INFO(get_logger(), ok ? "Sweep sequence complete." : "Sweep finished with errors.");
    return ok;
  }

private:
  static geometry_msgs::msg::Quaternion pitchQuaternion(double pitch)
  {
    geometry_msgs::msg::Quaternion q;
    q.y = std::sin(pitch / 2.0);
    q.w = std::cos(pitch / 2.0);
    return q;
  }

  static geometry_msgs::msg::Pose makePose(
    double x, double y, double z, const geometry_msgs::msg::Quaternion & q)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = q;
    return pose;
  }

  // Free-space (joint-space planned) move to a pose target.
  bool moveToPose(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const geometry_msgs::msg::Pose & pose, const std::string & label)
  {
    move_group.setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Planning failed: %s", label.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Moving: %s...", label.c_str());
    if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Execution failed: %s", label.c_str());
      return false;
    }
    return true;
  }

  // Straight-line Cartesian move of the TCP to a pose.
  bool sweepTo(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const geometry_msgs::msg::Pose & pose, const std::string & label)
  {
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction =
      move_group.computeCartesianPath({pose}, eef_step_, 0.0, trajectory);
    if (fraction < 0.99) {
      RCLCPP_ERROR(
        get_logger(), "Cartesian path for %s only covered %.0f%% of the line.",
        label.c_str(), fraction * 100.0);
      return false;
    }

    // computeCartesianPath returns full-speed timestamps; retime to the
    // configured velocity/acceleration scaling.
    robot_trajectory::RobotTrajectory retimed(
      move_group.getRobotModel(), move_group.getName());
    retimed.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(retimed, velocity_scaling_, acceleration_scaling_)) {
      RCLCPP_ERROR(get_logger(), "Time parameterization failed: %s", label.c_str());
      return false;
    }
    retimed.getRobotTrajectoryMsg(trajectory);

    RCLCPP_INFO(get_logger(), "Sweeping: %s...", label.c_str());
    if (move_group.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Execution failed: %s", label.c_str());
      return false;
    }
    return true;
  }

  void stopSpinner()
  {
    if (spinner_.joinable()) {
      executor_.cancel();
      spinner_.join();
    }
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
  double velocity_scaling_{0.3};
  double acceleration_scaling_{0.3};
  double eef_step_{0.01};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SweepMover>();
  const bool ok = node->run();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
