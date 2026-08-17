#include "main.hpp"

#include <cmath>
#include <memory>

#include "utils.hpp"

MainSweep::MainSweep()
    : Node("main_sweep",
          rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {}

MainSweep::~MainSweep() {
  stopSpinner();
}

bool MainSweep::run() {
  loadParameters();

  // Room check only uses positions, orientation does not matter here.
  geometry_msgs::msg::Quaternion identity;
  identity.w = 1.0;
  std::vector<Step> script = buildScript(identity);
  if (!checkAgainstRoom(script)) { return false; }

  if (!setup()) { return false; }
  bool ok = doMovement();
  if (ok) {
    RCLCPP_INFO(get_logger(), "Sweep complete.");
  } else {
    RCLCPP_INFO(get_logger(), "Sweep finished with errors.");
  }
  return ok;
}

void MainSweep::loadParameters() {
  planning_group_ = get_parameter_or<std::string>("planning_group", "ur_manipulator");
  velocity_scaling_ = get_parameter_or<double>("velocity_scaling", 0.05);
  acceleration_scaling_ = get_parameter_or<double>("acceleration_scaling", 0.1);
  sweep_x_ = get_parameter_or<double>("sweep_x", 1.0);
  sweep_z_ = get_parameter_or<double>("sweep_z", 0.35);
  sweep_y_start_ = get_parameter_or<double>("sweep_y_start", 0.9);
  sweep_y_end_ = get_parameter_or<double>("sweep_y_end", -0.9);
  eef_step_ = get_parameter_or<double>("eef_step", 0.01);
  park_joints_ = get_parameter_or<std::vector<double>>("park_joints", {});

  // Room bounds, same values room_publisher builds the collision boxes
  floor_z_ = get_parameter_or<double>("floor_z", -0.80);
  ceiling_z_ = get_parameter_or<double>("ceiling_z", 1.80);
  x_min_ = get_parameter_or<double>("x_min", -1.12);
  x_max_ = get_parameter_or<double>("x_max", 2.50);
  y_min_ = get_parameter_or<double>("y_min", -1.65);
  y_max_ = get_parameter_or<double>("y_max", 1.65);
  room_margin_ = get_parameter_or<double>("room_margin", 0.15);
}

std::vector<MainSweep::Step> MainSweep::buildScript(const geometry_msgs::msg::Quaternion &tcp_orientation) {

  // One orientation for the whole routine: whatever the TCP has in the
  // park pose. Only positions change, the TCP never rotates.
  geometry_msgs::msg::Quaternion q = tcp_orientation;

  double x = sweep_x_;
  double z = sweep_z_;
  double start = sweep_y_start_;
  double end = sweep_y_end_;

  std::vector<Step> script;
  script.push_back({Step::MOVE,  utils::makePose(x, start, z, q),           "sweep start"});
  script.push_back({Step::SWEEP, utils::makePose(x, end,   z, q),           "top sweep"});
  script.push_back({Step::MOVE,  utils::makePose(x - 0.1, start, z, q),     "return to start (offset line)"});
  script.push_back({Step::SWEEP, utils::makePose(x - 0.1, end,   z, q),     "second sweep"});
  script.push_back({Step::MOVE,  utils::makePose(x - 0.2, end,   z + 0.3, q), "final raised position"});
  return script;
}

// Refuse to run if any scripted TCP position is closer than room_margin
// to the floor, ceiling, or a wall. This catches bad config before the
// arm moves at all, instead of failing halfway through a sweep.
bool MainSweep::checkAgainstRoom(const std::vector<Step> &script) {
  bool all_ok = true;
  for (size_t i = 0; i < script.size(); i++) {
    double x = script[i].pose.position.x;
    double y = script[i].pose.position.y;
    double z = script[i].pose.position.z;
    std::string label = script[i].label;

    if (x < x_min_ + room_margin_ || x > x_max_ - room_margin_) {
      RCLCPP_ERROR(get_logger(), "'%s' x=%.2f is within %.2f m of an x wall [%.2f, %.2f].",
                   label.c_str(), x, room_margin_, x_min_, x_max_);
      all_ok = false;
    }
    if (y < y_min_ + room_margin_ || y > y_max_ - room_margin_) {
      RCLCPP_ERROR(get_logger(), "'%s' y=%.2f is within %.2f m of a y wall [%.2f, %.2f].",
                   label.c_str(), y, room_margin_, y_min_, y_max_);
      all_ok = false;
    }
    if (z < floor_z_ + room_margin_ || z > ceiling_z_ - room_margin_) {
      RCLCPP_ERROR(get_logger(), "'%s' z=%.2f is within %.2f m of the floor/ceiling [%.2f, %.2f].",
                   label.c_str(), z, room_margin_, floor_z_, ceiling_z_);
      all_ok = false;
    }
  }
  if (all_ok) {
    RCLCPP_INFO(get_logger(), "All %zu scripted poses are inside the room with %.2f m margin.",
                script.size(), room_margin_);
  }
  return all_ok;
}

bool MainSweep::setup() {

  // MoveGroupInterface needs the node spinning in the background for its
  // action clients and TF lookups.
  executor_.add_node(shared_from_this());
  spinner_ = std::thread([this]() { executor_.spin(); });

  move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group_);
  move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);

  std::string frame = move_group_->getPlanningFrame();
  std::string tcp_link = move_group_->getEndEffectorLink();
  RCLCPP_INFO(get_logger(), "Sweeping in frame '%s' with TCP link '%s' at %.0f%% speed.",
              frame.c_str(), tcp_link.c_str(), velocity_scaling_ * 100.0);

  // Give DDS discovery a moment to connect, otherwise the first planning
  // request can be lost.
  rclcpp::sleep_for(std::chrono::seconds(1));
  return true;
}

bool MainSweep::doMovement() {

  // First move to the parked pose (joint-space, exact) so every run
  // starts the routine from the same place no matter where the arm was.
  if (park_joints_.size() == 6) {
    if (!utils::moveToJoints(*move_group_, get_logger(), park_joints_, "move to park position")) { return false; }
  }

  // Hold the park pose's TCP orientation for the entire routine.
  geometry_msgs::msg::Quaternion tcp_orientation = move_group_->getCurrentPose().pose.orientation;

  std::vector<Step> script = buildScript(tcp_orientation);
  for (size_t i = 0; i < script.size(); i++) {
    Step step = script[i];
    bool ok = false;
    if (step.type == Step::MOVE) {
      ok = utils::moveToPose(*move_group_, get_logger(), step.pose, step.label);
    } else {
      ok = utils::sweepTo(*move_group_, get_logger(), eef_step_, velocity_scaling_, acceleration_scaling_, step.pose, step.label);
    }
    if (!ok) { return false; }
  }

  // Finish back in the resting (park) pose.
  if (park_joints_.size() == 6) {
    return utils::moveToJoints(*move_group_, get_logger(), park_joints_, "return to park");
  }
  return true;
}

void MainSweep::stopSpinner() {
  if (spinner_.joinable()) {
    executor_.cancel();
    spinner_.join();
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MainSweep>();
  bool ok = node->run();
  rclcpp::shutdown();
  if (ok) { return 0; }
  return 1;
}
