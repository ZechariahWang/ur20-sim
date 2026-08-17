#include "calibration_tests/sweep_mover.hpp"

#include <cmath>
#include <memory>

#include "utils.hpp"

SweepMover::SweepMover()
    : Node("sweep_mover",
          rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {}

SweepMover::~SweepMover() {
  stopSpinner();
}

bool SweepMover::run() {
  loadParameters();
  if (!setup()) { return false; }
  bool ok = doMovement();
  if (ok) {
    RCLCPP_INFO(get_logger(), "Sweep complete.");
  } else {
    RCLCPP_INFO(get_logger(), "Sweep finished with errors.");
  }
  return ok;
}

void SweepMover::loadParameters() {
  planning_group_ = get_parameter_or<std::string>("planning_group", "ur_manipulator");
  velocity_scaling_ = get_parameter_or<double>("velocity_scaling", 0.3);
  acceleration_scaling_ = get_parameter_or<double>("acceleration_scaling", 0.3);
  sweep_x_ = get_parameter_or<double>("sweep_x", 1.0);
  sweep_z_ = get_parameter_or<double>("sweep_z", 0.35);
  sweep_y_start_ = get_parameter_or<double>("sweep_y_start", 0.9);
  sweep_y_end_ = get_parameter_or<double>("sweep_y_end", -0.9);
  eef_step_ = get_parameter_or<double>("eef_step", 0.01);
  park_joints_ = get_parameter_or<std::vector<double>>("park_joints", {});
}

bool SweepMover::setup() {

  // MoveGroupInterface needs the node spinning in the background for its
  // action clients and TF lookups.
  executor_.add_node(shared_from_this());
  spinner_ = std::thread([this]() { executor_.spin(); });

  move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group_);
  move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);

  std::string frame = move_group_->getPlanningFrame();
  std::string tcp_link = move_group_->getEndEffectorLink();
  RCLCPP_INFO(get_logger(), "Sweeping in frame '%s' with TCP link '%s'.", frame.c_str(), tcp_link.c_str());

  // Give DDS discovery a moment to connect, otherwise the first planning
  // request can be lost.
  rclcpp::sleep_for(std::chrono::seconds(1));
  return true;
}

bool SweepMover::doMovement() {

  // First move to the parked pose (joint-space, exact) so every run
  // starts the routine from the same place no matter where the arm was.
  if (park_joints_.size() == 6) {
    if (!utils::moveToJoints(*move_group_, get_logger(), park_joints_, "move to park position")) { return false; }
  }

  // TCP orientations. In ROS base coordinates the open workspace is on
  // +x and the wall is behind the robot on -x (the pendant base frame
  // is rotated 180 deg about z, so pendant x is the opposite sign).
  geometry_msgs::msg::Quaternion down = utils::pitchQuaternion(M_PI);
  geometry_msgs::msg::Quaternion side = utils::pitchQuaternion(M_PI / 2.0);

  // Negative roll tilts left (toward the start), positive tilts right.
  geometry_msgs::msg::Quaternion oblique = utils::rollQuaternion(3.0 * M_PI / 4.0);

  double x = sweep_x_;
  double z = sweep_z_;
  double start = sweep_y_start_;
  double end = sweep_y_end_;

  std::vector<Step> script;
  script.push_back({Step::MOVE,  utils::makePose(x, start, z, down),                "sweep start"});
  script.push_back({Step::SWEEP, utils::makePose(x, end,   z, down),                "top sweep"});
  script.push_back({Step::MOVE,  utils::makePose(x - 0.1, start, z, side),          "return to start (side view)"});
  script.push_back({Step::SWEEP, utils::makePose(x - 0.1, end,   z, side),          "side sweep"});
  script.push_back({Step::MOVE,  utils::makePose(x - 0.2, end,   z + 0.3, oblique), "oblique view"});

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
  return true;
}

void SweepMover::stopSpinner() {
  if (spinner_.joinable()) {
    executor_.cancel();
    spinner_.join();
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SweepMover>();
  bool ok = node->run();
  rclcpp::shutdown();
  if (ok) { return 0; }
  return 1;
}
