#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
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

class RoomPublisher : public rclcpp::Node {

  public:

    RoomPublisher()
        : Node("room_publisher",
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {}

    void run() {
      loadParameters();
      std::vector<Box> boxes = buildRoom();
      publishMarkers(boxes);
      addToPlanningScene(boxes);
    }

  private:

    void loadParameters() {
      frame_id_ = get_parameter_or<std::string>("frame_id", "world");
      // Top surface of the floor. Slightly below zero so the robot base,
      // which sits at z = 0, does not touch it and count as a collision.
      floor_z_ = get_parameter_or<double>("floor_z", -0.01);
      // Bottom surface of the ceiling.
      ceiling_z_ = get_parameter_or<double>("ceiling_z", 2.5);
      // The room footprint is a square of this size, centered on the robot.
      room_size_ = get_parameter_or<double>("room_size", 4.0);
      // Thickness of the floor, ceiling, and wall slabs.
      thickness_ = get_parameter_or<double>("thickness", 0.1);
    }

    std::vector<Box> buildRoom() {
      double size = room_size_;
      double half = room_size_ / 2.0;
      double t = thickness_;
      double wall_height = ceiling_z_ - floor_z_;
      double wall_center_z = (ceiling_z_ + floor_z_) / 2.0;

      std::vector<Box> boxes;
      boxes.push_back({"floor",      0.0,       0.0,       floor_z_ - t / 2.0,   size, size, t});
      boxes.push_back({"ceiling",    0.0,       0.0,       ceiling_z_ + t / 2.0, size, size, t});
      boxes.push_back({"wall_front", half + t / 2.0,  0.0, wall_center_z,        t,    size, wall_height});
      boxes.push_back({"wall_back",  -half - t / 2.0, 0.0, wall_center_z,        t,    size, wall_height});
      boxes.push_back({"wall_left",  0.0,  half + t / 2.0,  wall_center_z,       size, t,    wall_height});
      boxes.push_back({"wall_right", 0.0,  -half - t / 2.0, wall_center_z,       size, t,    wall_height});
      return boxes;
    }

    // Latched markers so Foxglove still gets them if it connects later.
    void publishMarkers(const std::vector<Box> &boxes) {
      rclcpp::QoS qos(1);
      qos.transient_local();
      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("room_markers", qos);

      visualization_msgs::msg::MarkerArray markers;
      for (size_t i = 0; i < boxes.size(); i++) {
        Box box = boxes[i];
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.ns = "room";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = box.center_x;
        marker.pose.position.y = box.center_y;
        marker.pose.position.z = box.center_z;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = box.size_x;
        marker.scale.y = box.size_y;
        marker.scale.z = box.size_z;
        marker.color.r = 0.6;
        marker.color.g = 0.6;
        marker.color.b = 0.7;
        marker.color.a = 0.25;
        markers.markers.push_back(marker);
      }
      marker_pub_->publish(markers);
      RCLCPP_INFO(get_logger(), "Published %zu room markers on /room_markers.", boxes.size());
    }

    void addToPlanningScene(const std::vector<Box> &boxes) {
      std::vector<moveit_msgs::msg::CollisionObject> objects;
      for (size_t i = 0; i < boxes.size(); i++) {
        Box box = boxes[i];
        moveit_msgs::msg::CollisionObject object;
        object.header.frame_id = frame_id_;
        object.id = box.name;
        object.operation = moveit_msgs::msg::CollisionObject::ADD;

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions = {box.size_x, box.size_y, box.size_z};
        object.primitives.push_back(primitive);

        geometry_msgs::msg::Pose pose;
        pose.position.x = box.center_x;
        pose.position.y = box.center_y;
        pose.position.z = box.center_z;
        pose.orientation.w = 1.0;
        object.primitive_poses.push_back(pose);

        objects.push_back(object);
      }

      // move_group may still be starting, retry until it accepts the room.
      moveit::planning_interface::PlanningSceneInterface scene;
      for (int attempt = 0; attempt < 30; attempt++) {
        bool applied = scene.applyCollisionObjects(objects);
        if (applied) {
          RCLCPP_INFO(get_logger(), "Room added to the planning scene (%zu objects).", objects.size());
          return;
        }
        rclcpp::sleep_for(std::chrono::seconds(2));
      }
      RCLCPP_ERROR(get_logger(), "Could not add the room to the planning scene.");
    }

    std::string frame_id_;
    double floor_z_{-0.01};
    double ceiling_z_{2.5};
    double room_size_{4.0};
    double thickness_{0.1};
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RoomPublisher>();
  node->run();
  // Keep the node alive so the latched markers stay available.
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
