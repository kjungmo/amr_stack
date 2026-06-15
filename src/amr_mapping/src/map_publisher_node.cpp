// SPDX-License-Identifier: Apache-2.0
// map_publisher node: load a map YAML (param `map_yaml`) and latch it on /map
// as a nav_msgs/OccupancyGrid (TransientLocal, depth 1) in the `map` frame.
//
// Thin rclcpp wrapper around amr_mapping::load_map. All map parsing logic lives
// in the unit-tested library; this node only does ROS wiring.
#include <chrono>
#include <memory>
#include <string>

#include "amr_core/types.hpp"
#include "amr_mapping/map_io.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace amr_mapping {

namespace {

/// Convert an amr_core::OccupancyGrid into a nav_msgs/OccupancyGrid.
/// Grid row 0 = minimum-y (CONTRACT §2 item 3); ROS map_msg also stores row 0
/// at the origin, so the row order is preserved (no flip). `origin_(x,y)` is the
/// world corner of cell (0,0).
nav_msgs::msg::OccupancyGrid to_msg(const amr_core::OccupancyGrid& grid,
                                    const std::string& frame_id,
                                    const rclcpp::Time& stamp) {
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.info.resolution = static_cast<float>(grid.resolution);
  msg.info.width = static_cast<uint32_t>(grid.cols);
  msg.info.height = static_cast<uint32_t>(grid.rows);
  msg.info.origin.position.x = grid.origin_x;
  msg.info.origin.position.y = grid.origin_y;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;  // identity orientation.
  msg.data.assign(grid.data.begin(), grid.data.end());
  return msg;
}

}  // namespace

class MapPublisherNode : public rclcpp::Node {
 public:
  MapPublisherNode() : rclcpp::Node("map_publisher") {
    const std::string map_yaml = declare_parameter<std::string>("map_yaml", "");
    const std::string frame_id =
        declare_parameter<std::string>("frame_id", "map");

    if (map_yaml.empty()) {
      RCLCPP_FATAL(get_logger(),
                   "map_publisher: 'map_yaml' parameter is required");
      throw std::runtime_error("map_publisher: empty map_yaml");
    }

    amr_core::OccupancyGrid grid;
    try {
      grid = load_map(map_yaml);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "map_publisher: failed to load '%s': %s",
                   map_yaml.c_str(), e.what());
      throw;
    }

    // Latched QoS: reliable + TransientLocal, depth 1 (CONTRACT §4).
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable().transient_local();
    pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", qos);

    msg_ = to_msg(grid, frame_id, now());
    pub_->publish(msg_);
    RCLCPP_INFO(get_logger(),
                "map_publisher: latched /map (%u x %u @ %.3f m) from %s",
                msg_.info.width, msg_.info.height,
                static_cast<double>(msg_.info.resolution), map_yaml.c_str());

    // Re-publish periodically so late subscribers without TransientLocal still
    // receive the static map.
    timer_ = create_wall_timer(std::chrono::seconds(2), [this]() {
      msg_.header.stamp = now();
      pub_->publish(msg_);
    });
  }

 private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::OccupancyGrid msg_;
};

}  // namespace amr_mapping

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_mapping::MapPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
