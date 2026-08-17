// First-contact hardware test: rotate wrist_3 by a small angle and back.
// Joint-space only, no Cartesian targets, so the arm barely moves.

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

class TestMove : public rclcpp::Node {

  public:

    TestMove()
        : Node("test_move",
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {}

    ~TestMove() override {
      if (spinner_.joinable()) {
        executor_.cancel();
        spinner_.join();
      }
    }

    bool run() {
      executor_.add_node(shared_from_this());
      spinner_ = std::thread([this]() { executor_.spin(); });

      std::string group_name = get_parameter_or<std::string>("planning_group", "ur_manipulator");
      double wiggle = get_parameter_or<double>("wiggle_rad", 0.2);

      moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), group_name);
      move_group.setMaxVelocityScalingFactor(0.05);
      move_group.setMaxAccelerationScalingFactor(0.05);

      rclcpp::sleep_for(std::chrono::seconds(1));

      moveit::core::RobotStatePtr current = move_group.getCurrentState(10.0);
      const moveit::core::JointModelGroup *group = current->getJointModelGroup(group_name);
      std::vector<double> joints;
      current->copyJointGroupPositions(group, joints);

      RCLCPP_INFO(get_logger(), "Current joints: %.2f %.2f %.2f %.2f %.2f %.2f",
                  joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);

      // Move wrist_3 out by the wiggle angle, then back to where it was.
      std::vector<double> out = joints;
      out[5] = out[5] + wiggle;
      if (!moveTo(move_group, out, "wiggle out")) { return false; }
      if (!moveTo(move_group, joints, "wiggle back")) { return false; }

      RCLCPP_INFO(get_logger(), "Test complete. The robot moved and returned.");
      return true;
    }

  private:

    bool moveTo(moveit::planning_interface::MoveGroupInterface &move_group,
                const std::vector<double> &target, const std::string &label) {
      move_group.setJointValueTarget(target);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool planned = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (!planned) {
        RCLCPP_ERROR(get_logger(), "Planning failed: %s", label.c_str());
        return false;
      }
      RCLCPP_INFO(get_logger(), "Moving: %s...", label.c_str());
      bool executed = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (!executed) {
        RCLCPP_ERROR(get_logger(), "Execution failed: %s", label.c_str());
        return false;
      }
      return true;
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spinner_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestMove>();
  bool ok = node->run();
  rclcpp::shutdown();
  if (ok) { return 0; }
  return 1;
}
