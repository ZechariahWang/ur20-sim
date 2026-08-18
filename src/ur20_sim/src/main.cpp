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

  // Setup runs first because the angled script needs FK (move_group).
  if (!setup()) { return false; }

  // Room check only uses positions, orientation does not matter here.
  geometry_msgs::msg::Quaternion identity;
  identity.w = 1.0;
  std::vector<Step> script = buildScript(identity, identity);
  if (script.empty() && !park_only_) { return false; }
  if (!checkAgainstRoom(script)) { return false; }

  bool ok = doMovement();
  if (ok) {
    RCLCPP_INFO(get_logger(), "Sweep complete.");
  } else {
    RCLCPP_INFO(get_logger(), "Sweep finished with errors.");
  }
  return ok;
}

void MainSweep::loadParameters() {
  board_type_ = get_parameter_or<std::string>("board_type", "large");
  planning_group_ = get_parameter_or<std::string>("planning_group", "ur_manipulator");
  velocity_scaling_ = get_parameter_or<double>("velocity_scaling", 0.05);
  acceleration_scaling_ = get_parameter_or<double>("acceleration_scaling", 0.1);
  sweep_x_ = get_parameter_or<double>("sweep_x", 1.0);
  top_pass_height_ = get_parameter_or<double>("top_pass_height", 0.30);
  side_pass_height_ = get_parameter_or<double>("side_pass_height", 0.25);
  sweep_y_start_ = get_parameter_or<double>("sweep_y_start", 0.9);
  sweep_y_end_ = get_parameter_or<double>("sweep_y_end", -0.9);
  eef_step_ = get_parameter_or<double>("eef_step", 0.01);
  camera_length_ = get_parameter_or<double>("camera_length", 0.22);
  pose_pause_seconds_ = get_parameter_or<double>("pose_pause_seconds", 2.0);
  oblique_shift_x_ = get_parameter_or<double>("oblique_shift_x", 0.15);
  oblique_shift_y_ = get_parameter_or<double>("oblique_shift_y", 0.15);
  oblique_shift_z_ = get_parameter_or<double>("oblique_shift_z", 0.0);
  
  park_joints_ = get_parameter_or<std::vector<double>>("park_joints", {});
  park_only_ = get_parameter_or<bool>("park_only", false);
  side_view_joints_ = get_parameter_or<std::vector<double>>("side_view_joints", {});
  oblique_view_joints_ = get_parameter_or<std::vector<double>>("oblique_view_joints", {});
  angled_view_joints_ = get_parameter_or<std::vector<double>>("angled_view_joints", {});

  // Room bounds, same values room_publisher builds the collision boxes
  floor_z_ = get_parameter_or<double>("floor_z", -0.81);
  ceiling_z_ = get_parameter_or<double>("ceiling_z", 1.79);
  x_min_ = get_parameter_or<double>("x_min", -1.12);
  x_max_ = get_parameter_or<double>("x_max", 2.50);
  y_min_ = get_parameter_or<double>("y_min", -1.65);
  y_max_ = get_parameter_or<double>("y_max", 1.65);
  room_margin_ = get_parameter_or<double>("room_margin", 0.15);
  floor_margin_ = get_parameter_or<double>("floor_margin", 0.03);
}

std::vector<MainSweep::Step> MainSweep::buildScript(const geometry_msgs::msg::Quaternion &top_orientation, const geometry_msgs::msg::Quaternion &side_orientation) {
  if (board_type_ == "small") {
    RCLCPP_INFO(get_logger(), "Board type: small");
    return buildSmallBoardScript(top_orientation, side_orientation);
  }
  if (board_type_ == "angled") {
    RCLCPP_INFO(get_logger(), "Board type: angled");
    return buildAngledScript();
  }
  if (board_type_ != "large") {
    RCLCPP_WARN(get_logger(), "Unknown board_type '%s', using the large board sweep.", board_type_.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "Board type: large");
  }
  return buildLargeBoardScript(top_orientation, side_orientation);
}

std::vector<MainSweep::Step> MainSweep::buildLargeBoardScript(const geometry_msgs::msg::Quaternion &top_orientation, const geometry_msgs::msg::Quaternion &side_orientation) {

  geometry_msgs::msg::Quaternion q1 = top_orientation;
  geometry_msgs::msg::Quaternion q2 = side_orientation;

  double x = sweep_x_;
  double start = sweep_y_start_;
  double end = sweep_y_end_;
  double z_top = floor_z_ + top_pass_height_;
  double z_side = floor_z_ + side_pass_height_;

  std::vector<Step> script;
  script.push_back({Step::MOVE,  utils::makePose(x, start, z_top, q1),            "sweep start", {}});
  script.push_back({Step::SWEEP, utils::makePose(x, end,   z_top, q1),            "top sweep", {}});
  script.push_back({Step::MOVE,  utils::makePose(x - 0.1, start, z_side, q2),     "return to start (side view)", {}});
  script.push_back({Step::SWEEP, utils::makePose(x - 0.1, end,   z_side, q2),     "side sweep", {}});
  return script;
}

std::vector<MainSweep::Step> MainSweep::buildSmallBoardScript(const geometry_msgs::msg::Quaternion &top_orientation, const geometry_msgs::msg::Quaternion &side_orientation) {

  geometry_msgs::msg::Quaternion q1 = top_orientation;
  geometry_msgs::msg::Quaternion q2 = side_orientation;

  double x = sweep_x_;
  double y_center = (sweep_y_start_ + sweep_y_end_) / 2.0;
  double z_top = floor_z_ + top_pass_height_;
  double z_side = floor_z_ + side_pass_height_;

  std::vector<Step> script;
  script.push_back({Step::MOVE, utils::makePose(x, y_center, z_top, q1),         "top view", {}});
  // Side view stands 25 cm back from the board line so the camera body
  // cannot touch the wood. Shifted 15 cm toward -y.
  script.push_back({Step::MOVE, utils::makePose(x - 0.25, y_center - 0.15, z_side, q2), "side view", {}});
  return script;
}

std::vector<MainSweep::Step> MainSweep::buildAngledScript() {

  std::vector<Step> script;
  if (angled_view_joints_.size() != 6) {
    RCLCPP_ERROR(get_logger(), "Board type 'angled' needs angled_view_joints (6 values) in main_sweep.yaml.");
    return script;
  }

  geometry_msgs::msg::Pose flange_pose = utils::poseFromJoints(*move_group_, angled_view_joints_);
  geometry_msgs::msg::Pose camera_pose = utils::cameraPoseFromFlangePose(flange_pose, camera_length_);

  geometry_msgs::msg::Pose start_pose = camera_pose;
  start_pose.position.y = sweep_y_start_;
  geometry_msgs::msg::Pose end_pose = camera_pose;
  end_pose.position.y = sweep_y_end_;

  script.push_back({Step::MOVE,  start_pose, "angled sweep start", {}});
  script.push_back({Step::SWEEP, end_pose,   "angled sweep", {}});
  return script;
}

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
    if (z < floor_z_ + floor_margin_) {
      RCLCPP_ERROR(get_logger(), "'%s' z=%.2f is within %.2f m of the floor at %.2f.",
                   label.c_str(), z, floor_margin_, floor_z_);
      all_ok = false;
    }
    if (z > ceiling_z_ - room_margin_) {
      RCLCPP_ERROR(get_logger(), "'%s' z=%.2f is within %.2f m of the ceiling at %.2f.",
                   label.c_str(), z, room_margin_, ceiling_z_);
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

  executor_.add_node(shared_from_this());
  spinner_ = std::thread([this]() { executor_.spin(); });

  move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group_);
  move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);

  std::string frame = move_group_->getPlanningFrame();
  std::string tcp_link = move_group_->getEndEffectorLink();
  RCLCPP_INFO(get_logger(), "Sweeping in frame '%s' with TCP link '%s' at %.0f%% speed.",
              frame.c_str(), tcp_link.c_str(), velocity_scaling_ * 100.0);

  rclcpp::sleep_for(std::chrono::seconds(1));
  return true;
}

bool MainSweep::doMovement() {

  if (park_joints_.size() == 6) {
    if (!utils::moveToNearestJoints(*move_group_, get_logger(), park_joints_, "move to park position")) { return false; }
  }

  // "park" command from the webapp: just go to park, skip the sweeps.
  if (park_only_) { return true; }

  geometry_msgs::msg::Quaternion top_orientation = move_group_->getCurrentPose().pose.orientation;
  geometry_msgs::msg::Quaternion side_orientation = top_orientation;
  if (side_view_joints_.size() == 6) {
    side_orientation = utils::orientationFromJoints(*move_group_, side_view_joints_);
  }

  std::vector<Step> script = buildScript(top_orientation, side_orientation);

  geometry_msgs::msg::Pose last_flange_pose;
  for (size_t i = 0; i < script.size(); i++) {

    Step step = script[i];
    step.pose = utils::flangePoseFromCameraPose(step.pose, camera_length_);
    last_flange_pose = step.pose;

    bool ok = false;
    if (step.type == Step::MOVE) {
      if (step.seed.size() == 6) {
        ok = utils::moveToPoseSeeded(*move_group_, get_logger(), step.pose, step.seed, step.label);
      } else {
        ok = utils::moveToPose(*move_group_, get_logger(), step.pose, step.label);
      }
    } else {
      ok = utils::sweepTo(*move_group_, get_logger(), eef_step_, velocity_scaling_, acceleration_scaling_, step.pose, step.label);
    }
    if (!ok) { return false; }
    pauseAtPose(step.label);
  }

  if (oblique_view_joints_.size() == 6 && board_type_ != "angled") {
    // The tuned oblique spot: the jogged pose plus the configured shifts.
    geometry_msgs::msg::Pose oblique_base = utils::poseFromJoints(*move_group_, oblique_view_joints_);
    oblique_base.position.x = oblique_base.position.x + oblique_shift_x_;
    oblique_base.position.y = oblique_base.position.y + oblique_shift_y_;
    oblique_base.position.z = oblique_base.position.z + oblique_shift_z_;

    double y_center = (sweep_y_start_ + sweep_y_end_) / 2.0;
    double z_side = floor_z_ + side_pass_height_;
    geometry_msgs::msg::Pose reference_tip = utils::makePose(sweep_x_ - 0.25, y_center - 0.15, z_side, side_orientation);
    geometry_msgs::msg::Pose reference_flange = utils::flangePoseFromCameraPose(reference_tip, camera_length_);

    geometry_msgs::msg::Pose oblique_pose = oblique_base;
    oblique_pose.position.x = last_flange_pose.position.x + (oblique_base.position.x - reference_flange.position.x);
    oblique_pose.position.y = last_flange_pose.position.y + (oblique_base.position.y - reference_flange.position.y);
    oblique_pose.position.z = last_flange_pose.position.z + (oblique_base.position.z - reference_flange.position.z);

    // Go direct when the wrapped joint travel is modest; detour through
    // park only when a large wrist unwind makes the direct move swing.
    std::vector<double> wrapped = utils::nearestJointTarget(*move_group_, oblique_view_joints_);
    moveit::core::RobotStatePtr current = move_group_->getCurrentState(10.0);
    std::vector<double> current_joints;
    current->copyJointGroupPositions(current->getJointModelGroup(planning_group_), current_joints);
    double max_delta = 0.0;
    for (size_t i = 0; i < wrapped.size(); i++) {
      double delta = std::abs(wrapped[i] - current_joints[i]);
      if (delta > max_delta) { max_delta = delta; }
    }
    if (max_delta > 3.0 && park_joints_.size() == 6) {
      RCLCPP_INFO(get_logger(), "Large joint travel to oblique (%.1f rad), going via park.", max_delta);
      if (!utils::moveToNearestJoints(*move_group_, get_logger(), park_joints_, "via park position")) { return false; }
    }

    if (!utils::moveToPoseSeeded(*move_group_, get_logger(), oblique_pose, oblique_view_joints_, "oblique view")) { return false; }
    pauseAtPose("oblique view");
  }

  // back to start pos
  if (park_joints_.size() == 6) { return utils::moveToNearestJoints(*move_group_, get_logger(), park_joints_, "return to park"); }
  return true;
}

// Hold still at a reached pose (camera settle / capture time).
void MainSweep::pauseAtPose(const std::string &label) {
  if (pose_pause_seconds_ <= 0.0) { return; }
  RCLCPP_INFO(get_logger(), "Pausing %.1f s at '%s'.", pose_pause_seconds_, label.c_str());
  rclcpp::sleep_for(std::chrono::milliseconds((int)(pose_pause_seconds_ * 1000.0)));
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
