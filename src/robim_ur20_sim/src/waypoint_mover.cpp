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

    std::vector<std::string> waypoint_names;
    if (!get_parameter("waypoint_names", waypoint_names) || waypoint_names.empty()) {
      RCLCPP_ERROR(get_logger(), "No 'waypoint_names' parameter set — nothing to do.");
      return false;
    }

    moveit::planning_interface::MoveGroupInterface move_group(
      shared_from_this(), planning_group);
    move_group.setMaxVelocityScalingFactor(
      get_parameter_or<double>("velocity_scaling", 0.3));
    move_group.setMaxAccelerationScalingFactor(
      get_parameter_or<double>("acceleration_scaling", 0.3));

    RCLCPP_INFO(
      get_logger(), "Planning group '%s' ready. Visiting %zu waypoint(s)%s.",
      planning_group.c_str(), waypoint_names.size(), cycle ? " in a loop" : "");

    bool all_ok = true;
    do {
      for (const auto & name : waypoint_names) {
        if (!rclcpp::ok()) {
          break;
        }
        all_ok = moveToWaypoint(move_group, name) && all_ok;
      }
    } while (cycle && rclcpp::ok());

    RCLCPP_INFO(get_logger(), all_ok ? "All waypoints reached." : "Finished with errors.");
    return all_ok;
  }

private:
  // Plans and executes a single named waypoint from the parameter file.
  bool moveToWaypoint(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const std::string & name)
  {
    std::vector<double> positions;
    if (!get_parameter("waypoints." + name, positions)) {
      RCLCPP_ERROR(
        get_logger(), "Waypoint '%s' listed but not defined under 'waypoints.'", name.c_str());
      return false;
    }
    if (positions.size() != joint_names_.size()) {
      RCLCPP_ERROR(
        get_logger(), "Waypoint '%s' has %zu values, expected %zu.",
        name.c_str(), positions.size(), joint_names_.size());
      return false;
    }

    std::map<std::string, double> target;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      target[joint_names_[i]] = positions[i];
    }
    move_group.setJointValueTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Planning to waypoint '%s' failed.", name.c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "Moving to waypoint '%s'...", name.c_str());
    if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Execution of waypoint '%s' failed.", name.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Reached waypoint '%s'.", name.c_str());
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
