#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <rclcpp/rclcpp.hpp>

class SweepMover : public rclcpp::Node {

  public:

    SweepMover()
        : Node("sweep_mover",
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {}

    ~SweepMover() override { stopSpinner(); }

    bool run() {
      loadParameters();
      if (!setup()) { return false; }
      const bool ok = doMovement();
      RCLCPP_INFO(get_logger(), ok ? "Sweep complete." : "Sweep finished with errors.");
      return ok;
    }

  private:

    void loadParameters() {
      planning_group_ = get_parameter_or<std::string>("planning_group", "ur_manipulator");
      velocity_scaling_ = get_parameter_or<double>("velocity_scaling", 0.3);
      acceleration_scaling_ = get_parameter_or<double>("acceleration_scaling", 0.3);
      sweep_x_ = get_parameter_or<double>("sweep_x", 1.0);
      sweep_z_ = get_parameter_or<double>("sweep_z", 0.35);
      sweep_y_min_ = get_parameter_or<double>("sweep_y_min", -0.9);
      sweep_y_max_ = get_parameter_or<double>("sweep_y_max", 0.9);
      eef_step_ = get_parameter_or<double>("eef_step", 0.01);
    }

    bool setup() {
      executor_.add_node(shared_from_this());
      spinner_ = std::thread([this]() { executor_.spin(); });

      move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group_);
      move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
      move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);

      RCLCPP_INFO(get_logger(), "Sweeping in frame '%s' with TCP link '%s'.", move_group_->getPlanningFrame().c_str(), move_group_->getEndEffectorLink().c_str());
      rclcpp::sleep_for(std::chrono::seconds(1));

      return true;
    }

    bool doMovement() {

      const auto down = pitchQuaternion(M_PI); // TCP pointing straight down at the ground (180 deg pitch about the base Y axis).
      const auto side = pitchQuaternion(M_PI / 2.0); // TCP horizontal, pointing away from the robot (90 deg pitch).

      // move to the left end of the line.
      const auto left_down = makePose(sweep_x_, sweep_y_min_, sweep_z_, down);
      if (!moveToPose(left_down, "sweep start")) { return false; }

      // top sweep: left to right, TCP down.
      const auto right_down = makePose(sweep_x_, sweep_y_max_, sweep_z_, down);
      if (!sweepTo(right_down, "top sweep")) { return false; }

      // move back to the left end, now in the side view.
      const auto left_side = makePose(sweep_x_, sweep_y_min_, sweep_z_, side);
      if (!moveToPose(left_side, "return to start (side view)")) { return false; }

      // side sweep: left to right again, TCP sideways.
      const auto right_side = makePose(sweep_x_, sweep_y_max_, sweep_z_, side);
      return sweepTo(right_side, "side sweep");
    }

    bool moveToPose(const geometry_msgs::msg::Pose &pose, const std::string &label) {
      
      const auto current = move_group_->getCurrentState(10.0);
      const auto *group = current->getJointModelGroup(planning_group_);
      std::vector<double> current_joints;
      current->copyJointGroupPositions(group, current_joints);

      std::vector<double> best;
      double best_distance = std::numeric_limits<double>::infinity();
      moveit::core::RobotState candidate(*current);
      for (int attempt = 0; attempt < 20; ++attempt) {
        if (attempt > 0) { candidate.setToRandomPositions(group); }
        if (!candidate.setFromIK(group, pose, 0.1)) { continue; }

        std::vector<double> solution;
        candidate.copyJointGroupPositions(group, solution);
        wrapToNearest(solution, current_joints, group);

        double distance = 0.0;
        for (size_t i = 0; i < solution.size(); ++i) {
          distance += std::abs(solution[i] - current_joints[i]);
        }
        if (distance < best_distance) {
          best_distance = distance;
          best = solution;
        }
      }
      if (best.empty()) {
        RCLCPP_ERROR(get_logger(), "No reachable IK solution: %s", label.c_str());
        return false;
      }

      move_group_->setJointValueTarget(best);
      moveit::planning_interface::MoveGroupInterface::Plan plan;

      if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Planning failed: %s", label.c_str());
        return false;
      }
      RCLCPP_INFO(get_logger(), "Moving: %s (joint distance %.2f rad)...", label.c_str(), best_distance);
      if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Execution failed: %s", label.c_str());
        return false;
      }

      return true;
    }

    bool sweepTo(const geometry_msgs::msg::Pose &pose, const std::string &label) {

      moveit_msgs::msg::RobotTrajectory trajectory;
      const double fraction = move_group_->computeCartesianPath({pose}, eef_step_, 0.0, trajectory);

      if (fraction < 0.99) {
        RCLCPP_ERROR(get_logger(), "Cartesian path for %s only covered %.0f%% of the line.", label.c_str(), fraction * 100.0);
        return false;
      }

      robot_trajectory::RobotTrajectory retimed(move_group_->getRobotModel(), move_group_->getName());
      retimed.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);
      trajectory_processing::TimeOptimalTrajectoryGeneration totg;
      if (!totg.computeTimeStamps(retimed, velocity_scaling_, acceleration_scaling_)) {
        RCLCPP_ERROR(get_logger(), "Time parameterization failed: %s", label.c_str());
        return false;
      }

      retimed.getRobotTrajectoryMsg(trajectory);

      RCLCPP_INFO(get_logger(), "Sweeping: %s...", label.c_str());
      if (move_group_->execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Execution failed: %s", label.c_str());
        return false;
      }

      return true;
    }

    // Shift each joint by multiples of 2*pi to the equivalent angle closest to
    // its current value, staying inside the joint's limits.
    static void wrapToNearest(std::vector<double> &solution, const std::vector<double> &current, const moveit::core::JointModelGroup *group) {
      const auto &bounds = group->getActiveJointModelsBounds();
      for (size_t i = 0; i < solution.size(); ++i) {
        const auto &b = (*bounds[i])[0];
        double best = solution[i];
        for (int k = -2; k <= 2; ++k) {
          const double v = solution[i] + k * 2.0 * M_PI;
          if (v < b.min_position_ || v > b.max_position_) { continue; }
          if (std::abs(v - current[i]) < std::abs(best - current[i])) { best = v; }
        }
        solution[i] = best;
      }
    }

    static geometry_msgs::msg::Quaternion pitchQuaternion(double pitch) {
      geometry_msgs::msg::Quaternion q;
      q.y = std::sin(pitch / 2.0);
      q.w = std::cos(pitch / 2.0);
      return q;
    }

    static geometry_msgs::msg::Pose makePose(double x, double y, double z, const geometry_msgs::msg::Quaternion &q) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = x;
      pose.position.y = y;
      pose.position.z = z;
      pose.orientation = q;
      return pose;
    }

    void stopSpinner() {
      if (spinner_.joinable()) {
        executor_.cancel();
        spinner_.join();
      }
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spinner_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    std::string planning_group_;
    double velocity_scaling_{0.3};
    double acceleration_scaling_{0.3};
    double sweep_x_{1.0};
    double sweep_z_{0.35};
    double sweep_y_min_{-0.9};
    double sweep_y_max_{0.9};
    double eef_step_{0.01};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SweepMover>();
  const bool ok = node->run();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
