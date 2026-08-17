#ifndef UR20_SIM_ROOM_PUBLISHER_HPP
#define UR20_SIM_ROOM_PUBLISHER_HPP

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// One axis-aligned box of the room.
struct Box {
  std::string name;
  double center_x;
  double center_y;
  double center_z;
  double size_x;
  double size_y;
  double size_z;
};

// Publishes the real cell (floor, ceiling, walls, mounting pillar) as
// MoveIt collision objects and latched Foxglove markers.
class RoomPublisher : public rclcpp::Node {

  public:

    RoomPublisher();

    void run();

  private:

    void loadParameters();
    std::vector<Box> buildRoom();
    void publishMarkers(const std::vector<Box> &boxes);
    void addToPlanningScene(const std::vector<Box> &boxes);

    std::string frame_id_;
    double floor_z_{-0.81};
    double ceiling_z_{1.79};
    double x_min_{-1.12};
    double x_max_{2.50};
    double y_min_{-1.65};
    double y_max_{1.65};
    double thickness_{0.1};
    double pillar_size_{0.4};
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

#endif
