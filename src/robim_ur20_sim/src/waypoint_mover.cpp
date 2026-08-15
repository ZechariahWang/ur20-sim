// Moves the UR20 through a list of named joint-space waypoints using MoveIt 2.
// Waypoints are defined in config/waypoints.yaml.

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

class WaypointMover : public rclcpp::Node
{
public:
  WaypointMover()
  : Node(
      "waypoint_mover",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
  }

  ~WaypointMover() override
  {
    stopSpinner();
  }

  // Runs the full waypoint sequence. Returns true if every waypoint succeeded.
  bool run()
  {
    // MoveGroupInterface needs the node spinning for its action clients and TF.
    executor_.add_node(shared_from_this());
    spinner_ = std::thread([this]() { executor_.spin(); });

    const auto planning_group =
      get_parameter_or<std::string>("planning_group", "ur_manipulator");
    const bool cycle = get_parameter_or<bool>("cycle", false);

    joint_names_ = get_parameter_or<std::vector<std::string>>(
      "joint_names",
      {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"});

    // One flat array; every joint_names.size() values form one waypoint.
    std::vector<double> flat;
    if (!get_parameter("waypoints", flat) || flat.empty()) {
      RCLCPP_ERROR(get_logger(), "No 'waypoints' parameter set — nothing to do.");
      return false;
    }
    if (flat.size() % joint_names_.size() != 0) {
      RCLCPP_ERROR(
        get_logger(), "'waypoints' has %zu values, expected a multiple of %zu.",
        flat.size(), joint_names_.size());
      return false;
    }
    const size_t num_waypoints = flat.size() / joint_names_.size();

    moveit::planning_interface::MoveGroupInterface move_group(
      shared_from_this(), planning_group);
    move_group.setMaxVelocityScalingFactor(
      get_parameter_or<double>("velocity_scaling", 0.3));
    move_group.setMaxAccelerationScalingFactor(
      get_parameter_or<double>("acceleration_scaling", 0.3));

    RCLCPP_INFO(
      get_logger(), "Planning group '%s' ready. Visiting %zu waypoint(s)%s.",
      planning_group.c_str(), num_waypoints, cycle ? " in a loop" : "");

    bool all_ok = true;
    do {
      for (size_t i = 0; i < num_waypoints; ++i) {
        if (!rclcpp::ok()) {
          break;
        }
        const auto first = flat.begin() + i * joint_names_.size();
        const std::vector<double> positions(first, first + joint_names_.size());
        all_ok = moveToWaypoint(move_group, i, positions) && all_ok;
      }
    } while (cycle && rclcpp::ok());

    RCLCPP_INFO(get_logger(), all_ok ? "All waypoints reached." : "Finished with errors.");
    return all_ok;
  }

private:
  // Plans and executes a single waypoint.
  bool moveToWaypoint(
    moveit::planning_interface::MoveGroupInterface & move_group,
    size_t index, const std::vector<double> & positions)
  {
    std::map<std::string, double> target;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      target[joint_names_[i]] = positions[i];
    }
    move_group.setJointValueTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Planning to waypoint %zu failed.", index);
      return false;
    }

    RCLCPP_INFO(get_logger(), "Moving to waypoint %zu...", index);
    if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Execution of waypoint %zu failed.", index);
      return false;
    }
    RCLCPP_INFO(get_logger(), "Reached waypoint %zu.", index);
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
  std::vector<std::string> joint_names_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointMover>();
  const bool ok = node->run();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
