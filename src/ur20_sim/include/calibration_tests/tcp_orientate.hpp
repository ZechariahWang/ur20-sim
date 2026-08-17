#ifndef UR20_SIM_CALIBRATION_TESTS_TCP_ORIENTATE_HPP
#define UR20_SIM_CALIBRATION_TESTS_TCP_ORIENTATE_HPP

#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

// First-contact hardware test: park, then rotate wrist_3 by a small angle
// and back. Joint-space only, so the arm barely moves.
class TcpOrientate : public rclcpp::Node {

  public:

    TcpOrientate();
    ~TcpOrientate() override;

    bool run();

  private:

    bool moveTo(moveit::planning_interface::MoveGroupInterface &move_group, const std::vector<double> &target, const std::string &label);

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spinner_;
    std::vector<double> park_joints_;
};

#endif
