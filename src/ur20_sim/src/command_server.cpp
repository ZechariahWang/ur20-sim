#include "command_server.hpp"

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <csignal>
#include <memory>

CommandServer::CommandServer() : Node("command_server") {

  command_sub_ = create_subscription<std_msgs::msg::String>(
      "/webapp/command", 10,
      [this](const std_msgs::msg::String &msg) { onCommand(msg); });

  // Latched so the webapp sees the current status right after connecting.
  rclcpp::QoS qos(1);
  qos.transient_local();
  status_pub_ = create_publisher<std_msgs::msg::String>("/webapp/status", qos);

  timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { checkChild(); });

  trajectory_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      this, "/scaled_joint_trajectory_controller/follow_joint_trajectory");

  publishStatus("idle");
  RCLCPP_INFO(get_logger(), "Command server ready: publish 'sweep', 'park', or 'stop' to /webapp/command");
}

void CommandServer::onCommand(const std_msgs::msg::String &msg) {
  std::string command = msg.data;
  RCLCPP_INFO(get_logger(), "Received command: '%s'", command.c_str());

  if (command == "stop") {
    stopRoutine();
    return;
  }
  if (command == "sweep" || command == "park") {
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
    } else {
      execlp("ros2", "ros2", "launch", "ur20_sim", "main.launch.py", (char *)NULL);
    }
    _exit(127);
  }

  child_pid_ = pid;
  running_command_ = command;
  publishStatus("running: " + command);
}

void CommandServer::stopRoutine() {
  if (child_pid_ > 0) {
    // SIGINT the launch so the routine node shuts down like a Ctrl+C.
    kill(child_pid_, SIGINT);
    stopping_ = true;
    publishStatus("stopping: " + running_command_);
  } else {
    publishStatus("idle");
  }

  // Cancel the trajectory the controller is executing right now,
  // otherwise the arm finishes its current motion segment.
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
}

void CommandServer::publishStatus(const std::string &status) {
  std_msgs::msg::String msg;
  msg.data = status;
  status_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "Status: %s", status.c_str());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CommandServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
