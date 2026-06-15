// SPDX-License-Identifier: Apache-2.0
// Thin rclcpp wrapper around MonteCarloLocalizer.
//
// Subscribes /scan, /map (latched), /odom. On each scan it predicts the
// particle cloud from the accumulated odometry delta, corrects against the
// scan, and publishes /pose (PoseWithCovarianceStamped) and /particles
// (PoseArray). The map->odom tf correction is computed from the estimate
// (map->base_link) and the looked-up odom->base_link transform.
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "amr_core/config.hpp"
#include "amr_core/geometry.hpp"
#include "amr_core/types.hpp"
#include "amr_localization/mcl.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
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

class MclNode : public rclcpp::Node {
 public:
  MclNode() : rclcpp::Node("mcl_node") {
    // --- Parameters mirror amr_core::LocalizationConfig defaults. ----------
    amr_core::LocalizationConfig defaults;
    seed_ = this->declare_parameter<int>("seed", 42);
    cfg_.num_particles = this->declare_parameter<int>(
        "localization.num_particles", defaults.num_particles);
    cfg_.alphas = this->declare_parameter<std::vector<double>>(
        "localization.alphas", defaults.alphas);
    cfg_.init_std = this->declare_parameter<std::vector<double>>(
        "localization.init_std", defaults.init_std);
    cfg_.resample_neff_frac = this->declare_parameter<double>(
        "localization.resample_neff_frac", defaults.resample_neff_frac);
    cfg_.likelihood.sigma_hit = this->declare_parameter<double>(
        "localization.likelihood.sigma_hit", defaults.likelihood.sigma_hit);
    cfg_.likelihood.z_hit = this->declare_parameter<double>(
        "localization.likelihood.z_hit", defaults.likelihood.z_hit);
    cfg_.likelihood.z_rand = this->declare_parameter<double>(
        "localization.likelihood.z_rand", defaults.likelihood.z_rand);
    cfg_.likelihood.max_dist = this->declare_parameter<double>(
        "localization.likelihood.max_dist", defaults.likelihood.max_dist);
    cfg_.likelihood.beam_subsample = this->declare_parameter<int>(
        "localization.likelihood.beam_subsample",
        defaults.likelihood.beam_subsample);

    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");

    // Seed the particle cloud near a known start pose (Gaussian, init_std).
    // Set use_initial_pose=false for global localization (uniform over free
    // cells). Faithful to the Python MCL, which seeds around the spawn.
    use_init_pose_ = this->declare_parameter<bool>("use_initial_pose", true);
    init_pose_.x = this->declare_parameter<double>("initial_pose_x", 0.0);
    init_pose_.y = this->declare_parameter<double>("initial_pose_y", 0.0);
    init_pose_.theta = this->declare_parameter<double>("initial_pose_theta", 0.0);

    rng_.seed(static_cast<std::uint_fast32_t>(seed_));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // --- I/O -------------------------------------------------------------
    auto map_qos = rclcpp::QoS(1).transient_local().reliable();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", map_qos,
        std::bind(&MclNode::on_map, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS(),
        std::bind(&MclNode::on_odom, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&MclNode::on_scan, this, std::placeholders::_1));

    pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pose", rclcpp::QoS(10));
    particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/particles", rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(),
                "mcl_node up: %d particles, seed=%d (waiting for /map)",
                cfg_.num_particles, seed_);
  }

 private:
  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // NAV uses a static map that map_publisher latches and periodically
    // re-publishes for late subscribers. Initialise the filter from the FIRST
    // map only; re-initialising on every republish would reset the particle
    // cloud to the seed pose every couple of seconds and destroy the estimate.
    if (have_map_) {
      return;
    }
    amr_core::OccupancyGrid grid;
    grid.resolution = msg->info.resolution;
    grid.origin_x = msg->info.origin.position.x;
    grid.origin_y = msg->info.origin.position.y;
    grid.rows = static_cast<int>(msg->info.height);
    grid.cols = static_cast<int>(msg->info.width);
    grid.data.assign(msg->data.begin(), msg->data.end());

    const std::optional<amr_core::Pose2D> init =
        use_init_pose_ ? std::optional<amr_core::Pose2D>(init_pose_)
                       : std::nullopt;
    mcl_ = std::make_unique<amr_localization::MonteCarloLocalizer>(
        grid, cfg_, rng_, init);
    have_map_ = true;
    const std::string init_desc =
        use_init_pose_ ? "Gaussian@(" + std::to_string(init_pose_.x) + "," +
                             std::to_string(init_pose_.y) + ")"
                       : std::string("global");
    RCLCPP_INFO(this->get_logger(),
                "Map received: %dx%d @ %.3f m/cell; init %s",
                grid.cols, grid.rows, grid.resolution, init_desc.c_str());
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    last_odom_ = yaw_pose_from_quat(
        msg->pose.pose.position.x, msg->pose.pose.position.y,
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    have_odom_ = true;
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!have_map_ || !mcl_ || !have_odom_) {
      return;
    }

    // Accumulated odom delta in the robot frame since the last update.
    if (have_prev_odom_) {
      const amr_core::Pose2D delta =
          amr_core::pose_between(prev_odom_, last_odom_);
      mcl_->predict(delta);
    }
    prev_odom_ = last_odom_;
    have_prev_odom_ = true;

    // Build the amr_core scan and correct.
    amr_core::LaserScan scan;
    scan.angle_min = msg->angle_min;
    scan.angle_increment = msg->angle_increment;
    scan.range_min = msg->range_min;
    scan.range_max = msg->range_max;
    scan.stamp = rclcpp::Time(msg->header.stamp).seconds();
    scan.ranges.assign(msg->ranges.begin(), msg->ranges.end());
    mcl_->correct(scan);

    const amr_core::Pose2D est = mcl_->estimate();  // map->base_link
    publish_pose(est, msg->header.stamp);
    publish_particles(msg->header.stamp);
    broadcast_map_odom(est, msg->header.stamp);
  }

  void publish_pose(const amr_core::Pose2D& est, const rclcpp::Time& stamp) {
    geometry_msgs::msg::PoseWithCovarianceStamped m;
    m.header.stamp = stamp;
    m.header.frame_id = map_frame_;
    m.pose.pose.position.x = est.x;
    m.pose.pose.position.y = est.y;
    m.pose.pose.orientation = yaw_to_quat(est.theta);
    pose_pub_->publish(m);
  }

  void publish_particles(const rclcpp::Time& stamp) {
    geometry_msgs::msg::PoseArray m;
    m.header.stamp = stamp;
    m.header.frame_id = map_frame_;
    const auto& particles = mcl_->particles();
    m.poses.reserve(particles.size());
    for (const auto& p : particles) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = p.x;
      pose.position.y = p.y;
      pose.orientation = yaw_to_quat(p.theta);
      m.poses.push_back(pose);
    }
    particles_pub_->publish(m);
  }

  void broadcast_map_odom(const amr_core::Pose2D& est,
                          const rclcpp::Time& stamp) {
    // est is map->base_link. Look up odom->base_link from tf, then
    // map->odom = (map->base_link) (+) inv(odom->base_link).
    amr_core::Pose2D odom_base;
    try {
      const geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
          odom_frame_, base_frame_, tf2::TimePointZero);
      odom_base = yaw_pose_from_quat(
          tf.transform.translation.x, tf.transform.translation.y,
          tf.transform.rotation.x, tf.transform.rotation.y,
          tf.transform.rotation.z, tf.transform.rotation.w);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "odom->base_link tf unavailable: %s", ex.what());
      return;
    }

    // inv(odom->base_link) = base_link->odom = pose_between(odom_base, identity)
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

  // --- State ------------------------------------------------------------
  int seed_{42};
  amr_core::LocalizationConfig cfg_{};
  std::mt19937 rng_{};
  std::string base_frame_, odom_frame_, map_frame_;
  bool use_init_pose_{true};
  amr_core::Pose2D init_pose_{};

  std::unique_ptr<amr_localization::MonteCarloLocalizer> mcl_;
  bool have_map_{false};
  bool have_odom_{false};
  bool have_prev_odom_{false};
  amr_core::Pose2D last_odom_{};
  amr_core::Pose2D prev_odom_{};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MclNode>());
  rclcpp::shutdown();
  return 0;
}
