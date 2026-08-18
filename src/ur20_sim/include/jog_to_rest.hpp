#ifndef UR20_SIM_JOG_TO_REST_HPP
#define UR20_SIM_JOG_TO_REST_HPP

#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

class JogToRest : public rclcpp::Node {

  public:

    JogToRest();
    ~JogToRest() override;

    bool run();

  private:

    void loadParameters();
    bool setup();
    void stopSpinner();

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spinner_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    std::string planning_group_;
    double velocity_scaling_{0.05};
    double acceleration_scaling_{0.1};
    double camera_length_{0.22};
    std::vector<double> rest_joints_;
};

#endif
