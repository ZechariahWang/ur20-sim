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
      // All values are in the robot base frame (base origin = 0,0,0).
      // Top surface of the real floor, below the base when it is on a stand.
      floor_z_ = get_parameter_or<double>("floor_z", -0.80);
      // Bottom surface of the ceiling, above the base.
      ceiling_z_ = get_parameter_or<double>("ceiling_z", 1.80);
      // Walls: the reachable space spans x_min..x_max and y_min..y_max.
      x_min_ = get_parameter_or<double>("x_min", -2.50);
      x_max_ = get_parameter_or<double>("x_max", 1.12);
      y_min_ = get_parameter_or<double>("y_min", -1.50);
      y_max_ = get_parameter_or<double>("y_max", 1.50);
      // Thickness of the floor, ceiling, and wall slabs.
      thickness_ = get_parameter_or<double>("thickness", 0.1);
    }

    std::vector<Box> buildRoom() {
      double t = thickness_;
      double width_x = x_max_ - x_min_;
      double width_y = y_max_ - y_min_;
      double center_x = (x_max_ + x_min_) / 2.0;
      double center_y = (y_max_ + y_min_) / 2.0;
      double wall_height = ceiling_z_ - floor_z_;
      double wall_center_z = (ceiling_z_ + floor_z_) / 2.0;

      std::vector<Box> boxes;
      boxes.push_back({"floor",      center_x, center_y, floor_z_ - t / 2.0,   width_x, width_y, t});
      boxes.push_back({"ceiling",    center_x, center_y, ceiling_z_ + t / 2.0, width_x, width_y, t});
      boxes.push_back({"wall_back",  x_max_ + t / 2.0, center_y, wall_center_z, t, width_y, wall_height});
      boxes.push_back({"wall_front", x_min_ - t / 2.0, center_y, wall_center_z, t, width_y, wall_height});
      boxes.push_back({"wall_left",  center_x, y_max_ + t / 2.0, wall_center_z, width_x, t, wall_height});
      boxes.push_back({"wall_right", center_x, y_min_ - t / 2.0, wall_center_z, width_x, t, wall_height});
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
    double floor_z_{-0.80};
    double ceiling_z_{1.80};
    double x_min_{-2.50};
    double x_max_{1.12};
    double y_min_{-1.50};
    double y_max_{1.50};
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
