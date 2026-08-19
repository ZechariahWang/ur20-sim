#ifndef UR20_SIM_JOG_TO_REST_HPP
#define UR20_SIM_JOG_TO_REST_HPP

#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class JogToRest : public rclcpp::Node {

  public:

    JogToRest();
    ~JogToRest() override;

    bool run();

  private:

    void loadParameters();
    bool setup();
    void waitWhilePaused();
    void stopSpinner();

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spinner_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
    // True while the webapp holds the routine (latched /webapp/paused).
    std::atomic<bool> paused_{false};

    std::string planning_group_;
    double velocity_scaling_{0.05};
    double acceleration_scaling_{0.1};
    double camera_length_{0.22};
    std::vector<double> rest_joints_;
};

#endif
