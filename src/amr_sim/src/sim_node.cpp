// SPDX-License-Identifier: Apache-2.0
// sim_node: thin rclcpp wrapper around the amr_sim Simulator library.
//
// Timer at sim.dt advances the simulator under the latest /cmd_vel; publishes
// /scan (frame base_scan), /odom (odom->base_link), /ground_truth, broadcasts
// the dynamic odom->base_link tf and the static base_link->base_scan tf.
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_sim/simulator.hpp"
#include "amr_sim/world.hpp"

namespace {

geometry_msgs::msg::Quaternion yaw_to_quat(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion m;
  m.x = q.x();
  m.y = q.y();
  m.z = q.z();
  m.w = q.w();
  return m;
}

}  // namespace

class SimNode : public rclcpp::Node {
 public:
  SimNode() : rclcpp::Node("sim_node") {
    // --- Parameters (defaults from amr_core::*Config). ---
    amr_core::AmrConfig def;  // canonical defaults

    seed_ = declare_parameter<int>("seed", def.seed);

    cfg_.robot.radius =
        declare_parameter<double>("robot.radius", def.robot.radius);
    cfg_.robot.max_lin_vel =
        declare_parameter<double>("robot.max_lin_vel", def.robot.max_lin_vel);
    cfg_.robot.max_ang_vel =
        declare_parameter<double>("robot.max_ang_vel", def.robot.max_ang_vel);
    cfg_.robot.max_lin_acc =
        declare_parameter<double>("robot.max_lin_acc", def.robot.max_lin_acc);
    cfg_.robot.max_ang_acc =
        declare_parameter<double>("robot.max_ang_acc", def.robot.max_ang_acc);

    cfg_.lidar.num_beams =
        declare_parameter<int>("lidar.num_beams", def.lidar.num_beams);
    cfg_.lidar.angle_min =
        declare_parameter<double>("lidar.angle_min", def.lidar.angle_min);
    cfg_.lidar.angle_max =
        declare_parameter<double>("lidar.angle_max", def.lidar.angle_max);
    cfg_.lidar.range_min =
        declare_parameter<double>("lidar.range_min", def.lidar.range_min);
    cfg_.lidar.range_max =
        declare_parameter<double>("lidar.range_max", def.lidar.range_max);
    cfg_.lidar.noise_std =
        declare_parameter<double>("lidar.noise_std", def.lidar.noise_std);
    cfg_.lidar.scan_every =
        declare_parameter<int>("lidar.scan_every", def.lidar.scan_every);

    cfg_.sim.dt = declare_parameter<double>("sim.dt", def.sim.dt);
    cfg_.sim.world_file =
        declare_parameter<std::string>("world_file", def.sim.world_file);
    cfg_.sim.odom_noise.alpha_v = declare_parameter<double>(
        "sim.odom_noise.alpha_v", def.sim.odom_noise.alpha_v);
    cfg_.sim.odom_noise.alpha_w = declare_parameter<double>(
        "sim.odom_noise.alpha_w", def.sim.odom_noise.alpha_w);
    cfg_.sim.odom_noise.floor = declare_parameter<double>(
        "sim.odom_noise.floor", def.sim.odom_noise.floor);
    cfg_.seed = seed_;

    // Frame ids.
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    scan_frame_ = declare_parameter<std::string>("scan_frame", "base_scan");

    // --- Build the simulator. ---
    rng_ = std::make_unique<std::mt19937>(static_cast<std::uint32_t>(seed_));
    amr_sim::World world = amr_sim::World::from_yaml(cfg_.sim.world_file);
    sim_ = std::make_unique<amr_sim::Simulator>(std::move(world), cfg_, *rng_);

    // --- QoS: sensor data best-effort depth 5 (CONTRACT §4). ---
    const auto sensor_qos = rclcpp::SensorDataQoS();

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", sensor_qos);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", sensor_qos);
    gt_pub_ =
        create_publisher<nav_msgs::msg::Odometry>("ground_truth", sensor_qos);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          cmd_.v = msg->linear.x;
          cmd_.omega = msg->angular.z;
        });

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ =
        std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    // Static base_link -> base_scan (lidar mounted at robot center).
    geometry_msgs::msg::TransformStamped st;
    st.header.stamp = now();
    st.header.frame_id = base_frame_;
    st.child_frame_id = scan_frame_;
    st.transform.translation.x = 0.0;
    st.transform.translation.y = 0.0;
    st.transform.translation.z = 0.0;
    st.transform.rotation = yaw_to_quat(0.0);
    static_tf_broadcaster_->sendTransform(st);

    // Timer at sim.dt.
    const auto period = std::chrono::duration<double>(cfg_.sim.dt);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { on_timer(); });

    RCLCPP_INFO(get_logger(), "sim_node up: world='%s' seed=%d dt=%.4f",
                cfg_.sim.world_file.c_str(), seed_, cfg_.sim.dt);
  }

  ~SimNode() override {
    // Stop the timer before members are torn down so no callback can run
    // against half-destroyed publishers / tf broadcasters during shutdown.
    if (timer_) {
      timer_->cancel();
    }
  }

 private:
  void on_timer() {
    const amr_sim::SimStepResult res = sim_->step(cmd_);
    const rclcpp::Time stamp = now();

    // Dynamic tf: odom -> base_link (from drifting noisy odom pose).
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = res.odom_pose.x;
    tf.transform.translation.y = res.odom_pose.y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = yaw_to_quat(res.odom_pose.theta);
    tf_broadcaster_->sendTransform(tf);

    // /odom (odom -> base_link).
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = res.odom_pose.x;
    odom.pose.pose.position.y = res.odom_pose.y;
    odom.pose.pose.orientation = yaw_to_quat(res.odom_pose.theta);
    odom.twist.twist.linear.x = cmd_.v;
    odom.twist.twist.angular.z = cmd_.omega;
    odom_pub_->publish(odom);

    // /ground_truth (eval only; world -> base_link as the GT pose).
    nav_msgs::msg::Odometry gt;
    gt.header.stamp = stamp;
    gt.header.frame_id = "map";
    gt.child_frame_id = base_frame_;
    gt.pose.pose.position.x = res.ground_truth.x;
    gt.pose.pose.position.y = res.ground_truth.y;
    gt.pose.pose.orientation = yaw_to_quat(res.ground_truth.theta);
    gt_pub_->publish(gt);

    // /scan every scan_every steps.
    if (res.scan_ready) {
      sensor_msgs::msg::LaserScan ls;
      ls.header.stamp = stamp;
      ls.header.frame_id = scan_frame_;
      ls.angle_min = static_cast<float>(res.scan.angle_min);
      ls.angle_increment = static_cast<float>(res.scan.angle_increment);
      ls.angle_max = static_cast<float>(res.scan.angle_min +
                                        res.scan.angle_increment *
                                            res.scan.num_beams());
      ls.time_increment = 0.0f;
      ls.scan_time = static_cast<float>(cfg_.sim.dt * cfg_.lidar.scan_every);
      ls.range_min = static_cast<float>(res.scan.range_min);
      ls.range_max = static_cast<float>(res.scan.range_max);
      ls.ranges.resize(res.scan.ranges.size());
      for (std::size_t i = 0; i < res.scan.ranges.size(); ++i) {
        ls.ranges[i] = static_cast<float>(res.scan.ranges[i]);
      }
      scan_pub_->publish(ls);
    }
  }

  int seed_{42};
  amr_core::AmrConfig cfg_{};
  amr_core::Twist2D cmd_{};

  std::string odom_frame_;
  std::string base_frame_;
  std::string scan_frame_;

  std::unique_ptr<std::mt19937> rng_;
  std::unique_ptr<amr_sim::Simulator> sim_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gt_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimNode>());
  rclcpp::shutdown();
  return 0;
}
