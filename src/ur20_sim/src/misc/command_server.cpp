#include "command_server.hpp"

#include <sys/prctl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <csignal>
#include <memory>

CommandServer::CommandServer() : Node("command_server") {

  command_sub_ = create_subscription<std_msgs::msg::String>(
      "/webapp/command", 10,
      [this](const std_msgs::msg::String &msg) { onCommand(msg); });

  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState &msg) { latest_positions_ = msg.position; });

  // Latched so the webapp sees the current status right after connecting.
  rclcpp::QoS qos(1);
  qos.transient_local();
  status_pub_ = create_publisher<std_msgs::msg::String>("/webapp/status", qos);

  // Latched pause flag the routine nodes (main_sweep, jog_to_rest)
  // subscribe to. True = hold all motion until it turns false again.
  paused_pub_ = create_publisher<std_msgs::msg::Bool>("/webapp/paused", qos);
  publishPaused(false);

  timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
    checkChild();
    checkExternalMotion();
  });

  trajectory_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      this, "/scaled_joint_trajectory_controller/follow_joint_trajectory");

  publishStatus("idle");
  RCLCPP_INFO(get_logger(), "Command server ready: publish 'sweep', 'small', 'park', 'rest', 'pause', 'resume', or 'stop' to /webapp/command");
}

CommandServer::~CommandServer() {
  // Do not leave a running sweep launch behind when this node dies.
  if (child_pid_ > 0) {
    kill(child_pid_, SIGINT);
  }
}

void CommandServer::onCommand(const std_msgs::msg::String &msg) {
  std::string command = msg.data;
  RCLCPP_INFO(get_logger(), "Received command: '%s'", command.c_str());

  if (command == "stop") {
    stopRoutine();
    return;
  }
  if (command == "pause") {
    pauseRoutine();
    return;
  }
  if (command == "resume") {
    resumeRoutine();
    return;
  }
  if (command == "sweep" || command == "small" || command == "park" || command == "rest") {
    startRoutine(command);
    return;
  }
  publishStatus("error: unknown command '" + command + "'");
}

void CommandServer::startRoutine(const std::string &command) {
  if (child_pid_ > 0) {
    publishStatus("busy: '" + running_command_ + "' is still running");
    return;
  }

  int pid = fork();
  if (pid < 0) {
    publishStatus("error: could not start '" + command + "'");
    return;
  }
  if (pid == 0) {
    if (command == "park") {
      execlp("ros2", "ros2", "launch", "ur20_sim", "main.launch.py", "park_only:=true", (char *)NULL);
    } else if (command == "small") {
      execlp("ros2", "ros2", "launch", "ur20_sim", "main.launch.py", "board_type:=small", (char *)NULL);
    } else if (command == "rest") {
      execlp("ros2", "ros2", "launch", "ur20_sim", "rest.launch.py", (char *)NULL);
    } else {
      execlp("ros2", "ros2", "launch", "ur20_sim", "main.launch.py", (char *)NULL);
    }
    _exit(127);
  }

  child_pid_ = pid;
  running_command_ = command;
  routine_paused_ = false;
  publishPaused(false);
  publishStatus("running: " + command);
}

void CommandServer::pauseRoutine() {
  if (child_pid_ <= 0) {
    publishStatus("idle");
    return;
  }
  if (routine_paused_) {
    publishStatus("paused: " + running_command_);
    return;
  }
  routine_paused_ = true;

  publishPaused(true);
  if (trajectory_client_->action_server_is_ready()) {
    trajectory_client_->async_cancel_all_goals();
    RCLCPP_INFO(get_logger(), "Cancelled active trajectory goals for pause.");
  }
  publishStatus("paused: " + running_command_);
}

void CommandServer::resumeRoutine() {
  if (child_pid_ <= 0) {
    publishStatus("idle");
    return;
  }
  if (!routine_paused_) {
    publishStatus("running: " + running_command_);
    return;
  }
  routine_paused_ = false;
  publishPaused(false);
  publishStatus("running: " + running_command_);
}

void CommandServer::stopRoutine() {
  if (child_pid_ > 0) {
    if (routine_paused_) {
      routine_paused_ = false;
      publishPaused(false);
    }
    kill(child_pid_, SIGINT);
    stopping_ = true;
    publishStatus("stopping: " + running_command_);
  } else {
    publishStatus("idle");
  }

  if (trajectory_client_->action_server_is_ready()) {
    trajectory_client_->async_cancel_all_goals();
    RCLCPP_INFO(get_logger(), "Cancelled active trajectory goals.");
  }
}

void CommandServer::checkChild() {
  if (child_pid_ <= 0) { return; }

  int status = 0;
  int done = waitpid(child_pid_, &status, WNOHANG);
  if (done != child_pid_) { return; }

  bool clean = WIFEXITED(status) && WEXITSTATUS(status) == 0;
  if (stopping_) {
    publishStatus("stopped: " + running_command_);
  } else if (clean) {
    publishStatus("done: " + running_command_);
  } else {
    publishStatus("failed: " + running_command_);
  }
  child_pid_ = -1;
  stopping_ = false;
  running_command_ = "";
  if (routine_paused_) {
    routine_paused_ = false;
    publishPaused(false);
  }
  // Reset the motion baseline so the routine's final approach is not
  // mistaken for external motion on the next tick.
  last_checked_positions_ = latest_positions_;
  external_moving_ = false;
}

void CommandServer::checkExternalMotion() {
  std::vector<double> current = latest_positions_;
  if (current.empty()) { return; }
  if (last_checked_positions_.size() != current.size()) {
    last_checked_positions_ = current;
    return;
  }

  double max_delta = 0.0;
  for (size_t i = 0; i < current.size(); i++) {
    double delta = std::abs(current[i] - last_checked_positions_[i]);
    if (delta > max_delta) { max_delta = delta; }
  }
  last_checked_positions_ = current;

  // How far any joint moved in the last second. Real encoders jitter a
  // tiny amount while standing still, so require a real change.
  bool moving = max_delta > 0.002;

  if (child_pid_ > 0) {
    external_moving_ = false;
    return;
  }
  if (moving && !external_moving_) {
    external_moving_ = true;
    publishStatus("running: external");
  }
  if (!moving && external_moving_) {
    external_moving_ = false;
    publishStatus("idle");
  }
}

void CommandServer::publishStatus(const std::string &status) {
  std_msgs::msg::String msg;
  msg.data = status;
  status_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "Status: %s", status.c_str());
}

void CommandServer::publishPaused(bool paused) {
  std_msgs::msg::Bool msg;
  msg.data = paused;
  paused_pub_->publish(msg);
}

int main(int argc, char **argv) {
  prctl(PR_SET_PDEATHSIG, SIGTERM); // command server can never be left running as an orphan.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CommandServer>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
