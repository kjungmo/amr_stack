// SPDX-License-Identifier: Apache-2.0
// Thin rclcpp wrapper around ScanMatchingSlam.
//
// Subscribes /scan and /odom. On each scan it forms the odometry delta in the
// robot frame since the previous scan, runs the correlative scan match + map
// update, publishes the live /map (latched), and broadcasts the map->odom tf
// correction. A /save_map service writes the current map via amr_mapping
// map_io.
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "amr_core/config.hpp"
#include "amr_core/geometry.hpp"
#include "amr_core/types.hpp"
#include "amr_interfaces/srv/save_map.hpp"
#include "amr_mapping/map_io.hpp"
#include "amr_slam/scan_matching_slam.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace {

amr_core::Pose2D yaw_pose_from_quat(double x, double y, double qx, double qy,
                                    double qz, double qw) {
  tf2::Quaternion q(qx, qy, qz, qw);
  return amr_core::Pose2D{x, y, tf2::getYaw(q)};
}

geometry_msgs::msg::Quaternion yaw_to_quat(double theta) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

}  // namespace

class SlamNode : public rclcpp::Node {
 public:
  SlamNode() : rclcpp::Node("slam_node") {
    // --- Parameters mirror amr_core::SlamConfig / MappingConfig defaults. ---
    amr_core::SlamConfig slam_defaults;
    amr_core::MappingConfig map_defaults;

    slam_cfg_.keyframe_trans = this->declare_parameter<double>(
        "slam.keyframe_trans", slam_defaults.keyframe_trans);
    slam_cfg_.keyframe_rot = this->declare_parameter<double>(
        "slam.keyframe_rot", slam_defaults.keyframe_rot);
    slam_cfg_.min_motion = this->declare_parameter<double>(
        "slam.min_motion", slam_defaults.min_motion);
    slam_cfg_.match_beams = this->declare_parameter<int>(
        "slam.match_beams", slam_defaults.match_beams);
    slam_cfg_.coarse_window_xy = this->declare_parameter<double>(
        "slam.coarse_window_xy", slam_defaults.coarse_window_xy);
    slam_cfg_.coarse_step_xy = this->declare_parameter<double>(
        "slam.coarse_step_xy", slam_defaults.coarse_step_xy);
    slam_cfg_.coarse_window_theta = this->declare_parameter<double>(
        "slam.coarse_window_theta", slam_defaults.coarse_window_theta);
    slam_cfg_.coarse_step_theta = this->declare_parameter<double>(
        "slam.coarse_step_theta", slam_defaults.coarse_step_theta);
    slam_cfg_.fine_step_xy = this->declare_parameter<double>(
        "slam.fine_step_xy", slam_defaults.fine_step_xy);
    slam_cfg_.fine_step_theta = this->declare_parameter<double>(
        "slam.fine_step_theta", slam_defaults.fine_step_theta);
    slam_cfg_.blur_sigma_cells = this->declare_parameter<double>(
        "slam.blur_sigma_cells", slam_defaults.blur_sigma_cells);
    slam_cfg_.min_match_score = this->declare_parameter<double>(
        "slam.min_match_score", slam_defaults.min_match_score);

    mapping_cfg_.resolution = this->declare_parameter<double>(
        "mapping.resolution", map_defaults.resolution);
    mapping_cfg_.l_occ =
        this->declare_parameter<double>("mapping.l_occ", map_defaults.l_occ);
    mapping_cfg_.l_free =
        this->declare_parameter<double>("mapping.l_free", map_defaults.l_free);
    mapping_cfg_.l_clamp = this->declare_parameter<double>(
        "mapping.l_clamp", map_defaults.l_clamp);
    mapping_cfg_.occupied_thresh = this->declare_parameter<double>(
        "mapping.occupied_thresh", map_defaults.occupied_thresh);
    mapping_cfg_.free_thresh = this->declare_parameter<double>(
        "mapping.free_thresh", map_defaults.free_thresh);
    mapping_cfg_.beam_subsample = this->declare_parameter<int>(
        "mapping.beam_subsample", map_defaults.beam_subsample);

    // Map extent and initial pose (the world origin is fixed at (0, 0)).
    const double size_x = this->declare_parameter<double>("map_size_x", 12.0);
    const double size_y = this->declare_parameter<double>("map_size_y", 9.0);
    const double init_x = this->declare_parameter<double>("initial_x", 0.0);
    const double init_y = this->declare_parameter<double>("initial_y", 0.0);
    const double init_theta =
        this->declare_parameter<double>("initial_theta", 0.0);

    base_frame_ =
        this->declare_parameter<std::string>("base_frame", "base_link");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");

    slam_ = std::make_unique<amr_slam::ScanMatchingSlam>(
        slam_cfg_, mapping_cfg_, std::make_pair(size_x, size_y),
        amr_core::Pose2D{init_x, init_y, init_theta});

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // --- I/O -------------------------------------------------------------
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS(),
        std::bind(&SlamNode::on_odom, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&SlamNode::on_scan, this, std::placeholders::_1));

    auto map_qos = rclcpp::QoS(1).transient_local().reliable();
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",
                                                                    map_qos);

    save_srv_ = this->create_service<amr_interfaces::srv::SaveMap>(
        "/save_map", std::bind(&SlamNode::on_save_map, this,
                               std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
                "slam_node up: map %.1fx%.1f m @ %.3f m/cell", size_x, size_y,
                mapping_cfg_.resolution);
  }

 private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    last_odom_ = yaw_pose_from_quat(
        msg->pose.pose.position.x, msg->pose.pose.position.y,
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    have_odom_ = true;
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!have_odom_) {
      return;
    }

    // Odometry delta in the robot frame since the previous scan.
    amr_core::Pose2D delta{0.0, 0.0, 0.0};
    if (have_prev_odom_) {
      delta = amr_core::pose_between(prev_odom_, last_odom_);
    }
    prev_odom_ = last_odom_;
    have_prev_odom_ = true;

    amr_core::LaserScan scan;
    scan.angle_min = msg->angle_min;
    scan.angle_increment = msg->angle_increment;
    scan.range_min = msg->range_min;
    scan.range_max = msg->range_max;
    scan.stamp = rclcpp::Time(msg->header.stamp).seconds();
    scan.ranges.assign(msg->ranges.begin(), msg->ranges.end());

    const amr_core::Pose2D est =
        slam_->process(delta, std::optional<amr_core::LaserScan>(scan));

    publish_map(msg->header.stamp);
    broadcast_map_odom(est, msg->header.stamp);
  }

  void publish_map(const rclcpp::Time& stamp) {
    const amr_core::OccupancyGrid grid = slam_->get_map();
    nav_msgs::msg::OccupancyGrid m;
    m.header.stamp = stamp;
    m.header.frame_id = map_frame_;
    m.info.resolution = static_cast<float>(grid.resolution);
    m.info.width = static_cast<unsigned int>(grid.cols);
    m.info.height = static_cast<unsigned int>(grid.rows);
    m.info.origin.position.x = grid.origin_x;
    m.info.origin.position.y = grid.origin_y;
    m.info.origin.position.z = 0.0;
    m.info.origin.orientation.w = 1.0;
    m.data.assign(grid.data.begin(), grid.data.end());
    map_pub_->publish(m);
  }

  void broadcast_map_odom(const amr_core::Pose2D& est,
                          const rclcpp::Time& stamp) {
    // est is map->base_link. Look up odom->base_link from tf, then
    // map->odom = (map->base_link) (+) inv(odom->base_link).
    amr_core::Pose2D odom_base;
    try {
      const geometry_msgs::msg::TransformStamped tf =
          tf_buffer_->lookupTransform(odom_frame_, base_frame_,
                                      tf2::TimePointZero);
      odom_base = yaw_pose_from_quat(
          tf.transform.translation.x, tf.transform.translation.y,
          tf.transform.rotation.x, tf.transform.rotation.y,
          tf.transform.rotation.z, tf.transform.rotation.w);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "odom->base_link tf unavailable: %s", ex.what());
      return;
    }

    const amr_core::Pose2D identity{0.0, 0.0, 0.0};
    const amr_core::Pose2D base_odom =
        amr_core::pose_between(odom_base, identity);
    const amr_core::Pose2D map_odom = amr_core::pose_compose(est, base_odom);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = map_frame_;
    t.child_frame_id = odom_frame_;
    t.transform.translation.x = map_odom.x;
    t.transform.translation.y = map_odom.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation = yaw_to_quat(map_odom.theta);
    tf_broadcaster_->sendTransform(t);
  }

  void on_save_map(
      const std::shared_ptr<amr_interfaces::srv::SaveMap::Request> req,
      std::shared_ptr<amr_interfaces::srv::SaveMap::Response> res) {
    try {
      const amr_core::OccupancyGrid grid = slam_->get_map();
      const std::pair<std::string, std::string> paths =
          amr_mapping::save_map(grid, req->path);
      res->success = true;
      res->message = "wrote " + paths.first + " + " + paths.second;
      RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
    } catch (const std::exception& ex) {
      res->success = false;
      res->message = std::string("save_map failed: ") + ex.what();
      RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
    }
  }

  // --- State ------------------------------------------------------------
  amr_core::SlamConfig slam_cfg_{};
  amr_core::MappingConfig mapping_cfg_{};
  std::unique_ptr<amr_slam::ScanMatchingSlam> slam_;
  std::string base_frame_, odom_frame_, map_frame_;

  bool have_odom_{false};
  bool have_prev_odom_{false};
  amr_core::Pose2D last_odom_{};
  amr_core::Pose2D prev_odom_{};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Service<amr_interfaces::srv::SaveMap>::SharedPtr save_srv_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamNode>());
  rclcpp::shutdown();
  return 0;
}
