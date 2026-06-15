// SPDX-License-Identifier: Apache-2.0
// navigator_node: thin rclcpp wrapper around the Navigator FSM library.
//
// - Action server: amr_interfaces/action/NavigateToGoal (NavigateToGoal).
// - Sub /goal_pose (geometry_msgs/PoseStamped, RViz "2D Goal Pose").
// - Sub /map (nav_msgs/OccupancyGrid, latched: reliable + TransientLocal).
// - Pose from tf map->base_link.
// - Pub /cmd_vel (geometry_msgs/Twist), /plan (nav_msgs/Path, frame map).
// - Runs the FSM on a timer; publishes action feedback (state,
//   distance_remaining) and result (success, final_error, final_state).
//
// The costmap is built from the latched /map with inflation radius
// robot.radius + 0.06 (the NAV planning margin from architecture.md).
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_planning/costmap.hpp"
#include "amr_interfaces/action/navigate_to_goal.hpp"

#include "amr_navigation/navigator.hpp"

namespace amr_navigation {

namespace {
// Extra clearance (m) added to the robot radius when inflating the NAV planning
// costmap. Mirrors amr/runtime/app.py::_NAV_PLANNING_MARGIN. See CONTRACT §6.
constexpr double kNavPlanningMargin = 0.06;
}  // namespace

using NavigateToGoal = amr_interfaces::action::NavigateToGoal;
using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToGoal>;

class NavigatorNode : public rclcpp::Node {
 public:
  NavigatorNode() : rclcpp::Node("navigator_node") {
    declare_params();
    load_config_from_params();

    navigator_ = std::make_unique<Navigator>(Navigator::make_default(
        nav_cfg_, astar_cfg_, dwa_cfg_, robot_cfg_));

    // tf: map -> base_link.
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publishers.
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    plan_pub_ = create_publisher<nav_msgs::msg::Path>("/plan", 10);

    // /map latched: reliable + TransientLocal, depth 1.
    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", map_qos,
        std::bind(&NavigatorNode::on_map, this, std::placeholders::_1));

    // /goal_pose (RViz 2D Goal Pose).
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        std::bind(&NavigatorNode::on_goal_pose, this, std::placeholders::_1));

    // Action server.
    action_server_ = rclcpp_action::create_server<NavigateToGoal>(
        this, "navigate_to_goal",
        std::bind(&NavigatorNode::handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&NavigatorNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&NavigatorNode::handle_accepted, this,
                  std::placeholders::_1));

    // FSM timer at sim dt.
    const auto period =
        std::chrono::duration<double>(sim_dt_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&NavigatorNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "navigator_node ready");
  }

 private:
  // ----------------------------------------------------------- param loading
  void declare_params() {
    declare_parameter<std::string>("config_file", "");
    declare_parameter<int>("seed", 42);
    declare_parameter<double>("sim.dt", 0.05);
    // robot.*
    declare_parameter<double>("robot.radius", robot_cfg_.radius);
    declare_parameter<double>("robot.max_lin_vel", robot_cfg_.max_lin_vel);
    declare_parameter<double>("robot.max_ang_vel", robot_cfg_.max_ang_vel);
    declare_parameter<double>("robot.max_lin_acc", robot_cfg_.max_lin_acc);
    declare_parameter<double>("robot.max_ang_acc", robot_cfg_.max_ang_acc);
    // nav.*
    declare_parameter<double>("nav.goal_tol_xy", nav_cfg_.goal_tol_xy);
    declare_parameter<double>("nav.replan_period", nav_cfg_.replan_period);
    declare_parameter<double>("nav.path_block_check_dist",
                              nav_cfg_.path_block_check_dist);
    declare_parameter<int>("nav.max_recoveries", nav_cfg_.max_recoveries);
    declare_parameter<double>("nav.recovery_rotate_speed",
                              nav_cfg_.recovery_rotate_speed);
    declare_parameter<double>("nav.recovery_backup_dist",
                              nav_cfg_.recovery_backup_dist);
    declare_parameter<double>("nav.recovery_backup_speed",
                              nav_cfg_.recovery_backup_speed);
    // planning.costmap.*
    declare_parameter<int>("planning.costmap.occupied_thresh",
                           costmap_cfg_.occupied_thresh);
    declare_parameter<bool>("planning.costmap.unknown_is_lethal",
                            costmap_cfg_.unknown_is_lethal);
    declare_parameter<double>("planning.costmap.inflation_radius",
                              costmap_cfg_.inflation_radius);
    declare_parameter<double>("planning.costmap.cost_decay",
                              costmap_cfg_.cost_decay);
    // planning.astar.*
    declare_parameter<double>("planning.astar.w_cost", astar_cfg_.w_cost);
    declare_parameter<bool>("planning.astar.simplify", astar_cfg_.simplify);
    // planning.dwa.*
    declare_parameter<double>("planning.dwa.sim_time", dwa_cfg_.sim_time);
    declare_parameter<double>("planning.dwa.sim_dt", dwa_cfg_.sim_dt);
    declare_parameter<int>("planning.dwa.v_samples", dwa_cfg_.v_samples);
    declare_parameter<int>("planning.dwa.w_samples", dwa_cfg_.w_samples);
    declare_parameter<double>("planning.dwa.lookahead", dwa_cfg_.lookahead);
    declare_parameter<double>("planning.dwa.w_progress", dwa_cfg_.w_progress);
    declare_parameter<double>("planning.dwa.w_heading", dwa_cfg_.w_heading);
    declare_parameter<double>("planning.dwa.w_clearance", dwa_cfg_.w_clearance);
    declare_parameter<double>("planning.dwa.w_velocity", dwa_cfg_.w_velocity);
  }

  void load_config_from_params() {
    // Optionally seed from a config_file, then override with individual params.
    const std::string config_file =
        get_parameter("config_file").as_string();
    if (!config_file.empty()) {
      amr_core::AmrConfig full = amr_core::load_config(config_file);
      robot_cfg_ = full.robot;
      nav_cfg_ = full.nav;
      costmap_cfg_ = full.planning.costmap;
      astar_cfg_ = full.planning.astar;
      dwa_cfg_ = full.planning.dwa;
      sim_dt_ = full.sim.dt;
    }

    sim_dt_ = get_parameter("sim.dt").as_double();
    robot_cfg_.radius = get_parameter("robot.radius").as_double();
    robot_cfg_.max_lin_vel = get_parameter("robot.max_lin_vel").as_double();
    robot_cfg_.max_ang_vel = get_parameter("robot.max_ang_vel").as_double();
    robot_cfg_.max_lin_acc = get_parameter("robot.max_lin_acc").as_double();
    robot_cfg_.max_ang_acc = get_parameter("robot.max_ang_acc").as_double();

    nav_cfg_.goal_tol_xy = get_parameter("nav.goal_tol_xy").as_double();
    nav_cfg_.replan_period = get_parameter("nav.replan_period").as_double();
    nav_cfg_.path_block_check_dist =
        get_parameter("nav.path_block_check_dist").as_double();
    nav_cfg_.max_recoveries =
        static_cast<int>(get_parameter("nav.max_recoveries").as_int());
    nav_cfg_.recovery_rotate_speed =
        get_parameter("nav.recovery_rotate_speed").as_double();
    nav_cfg_.recovery_backup_dist =
        get_parameter("nav.recovery_backup_dist").as_double();
    nav_cfg_.recovery_backup_speed =
        get_parameter("nav.recovery_backup_speed").as_double();

    costmap_cfg_.occupied_thresh = static_cast<int>(
        get_parameter("planning.costmap.occupied_thresh").as_int());
    costmap_cfg_.unknown_is_lethal =
        get_parameter("planning.costmap.unknown_is_lethal").as_bool();
    costmap_cfg_.inflation_radius =
        get_parameter("planning.costmap.inflation_radius").as_double();
    costmap_cfg_.cost_decay =
        get_parameter("planning.costmap.cost_decay").as_double();

    astar_cfg_.w_cost = get_parameter("planning.astar.w_cost").as_double();
    astar_cfg_.simplify = get_parameter("planning.astar.simplify").as_bool();

    dwa_cfg_.sim_time = get_parameter("planning.dwa.sim_time").as_double();
    dwa_cfg_.sim_dt = get_parameter("planning.dwa.sim_dt").as_double();
    dwa_cfg_.v_samples =
        static_cast<int>(get_parameter("planning.dwa.v_samples").as_int());
    dwa_cfg_.w_samples =
        static_cast<int>(get_parameter("planning.dwa.w_samples").as_int());
    dwa_cfg_.lookahead = get_parameter("planning.dwa.lookahead").as_double();
    dwa_cfg_.w_progress = get_parameter("planning.dwa.w_progress").as_double();
    dwa_cfg_.w_heading = get_parameter("planning.dwa.w_heading").as_double();
    dwa_cfg_.w_clearance =
        get_parameter("planning.dwa.w_clearance").as_double();
    dwa_cfg_.w_velocity = get_parameter("planning.dwa.w_velocity").as_double();
  }

  // ------------------------------------------------------------ subscriptions
  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    amr_core::OccupancyGrid grid;
    grid.resolution = msg->info.resolution;
    grid.origin_x = msg->info.origin.position.x;
    grid.origin_y = msg->info.origin.position.y;
    grid.rows = static_cast<int>(msg->info.height);
    grid.cols = static_cast<int>(msg->info.width);
    grid.data.assign(msg->data.begin(), msg->data.end());

    std::lock_guard<std::mutex> lock(mutex_);
    // Build the planning costmap once from the loaded static map, inflating with
    // robot.radius + NAV planning margin.
    costmap_ = std::make_shared<amr_planning::Costmap>(
        grid, costmap_cfg_, robot_cfg_.radius + kNavPlanningMargin);
    RCLCPP_INFO(get_logger(), "navigator_node: received /map (%dx%d)",
                grid.cols, grid.rows);
  }

  void on_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    amr_core::Pose2D goal;
    goal.x = msg->pose.position.x;
    goal.y = msg->pose.position.y;
    goal.theta = tf2::getYaw(msg->pose.orientation);
    std::lock_guard<std::mutex> lock(mutex_);
    navigator_->set_goal(goal);
    RCLCPP_INFO(get_logger(), "navigator_node: goal (%.2f, %.2f) via /goal_pose",
                goal.x, goal.y);
  }

  // ---------------------------------------------------------- action handlers
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& /*uuid*/,
      std::shared_ptr<const NavigateToGoal::Goal> /*goal*/) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandle> /*goal_handle*/) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    std::lock_guard<std::mutex> lock(mutex_);
    // Preempt any prior goal.
    active_goal_handle_ = goal_handle;
    const auto goal_msg = goal_handle->get_goal();
    amr_core::Pose2D goal;
    goal.x = goal_msg->goal.pose.position.x;
    goal.y = goal_msg->goal.pose.position.y;
    goal.theta = tf2::getYaw(goal_msg->goal.pose.orientation);
    navigator_->set_goal(goal);
    RCLCPP_INFO(get_logger(),
                "navigator_node: accepted NavigateToGoal (%.2f, %.2f)", goal.x,
                goal.y);
  }

  // -------------------------------------------------------------------- timer
  bool lookup_pose(amr_core::Pose2D& out) {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "navigator_node: tf map->base_link unavailable: %s",
                           ex.what());
      return false;
    }
    out.x = tf.transform.translation.x;
    out.y = tf.transform.translation.y;
    out.theta = tf2::getYaw(tf.transform.rotation);
    return true;
  }

  void on_timer() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!costmap_) {
      return;  // no map yet
    }
    if (navigator_->state() == NavState::IDLE) {
      return;  // nothing to do
    }

    amr_core::Pose2D pose;
    if (!lookup_pose(pose)) {
      return;
    }

    // Caller sim clock = node clock (seconds).
    const double now = now_seconds();
    const amr_core::Twist2D vel{last_cmd_.v, last_cmd_.omega};

    const NavState prev = navigator_->state();
    amr_core::Twist2D cmd = navigator_->update(pose, vel, *costmap_, now);
    last_cmd_ = cmd;

    // Publish command.
    geometry_msgs::msg::Twist tw;
    tw.linear.x = cmd.v;
    tw.angular.z = cmd.omega;
    cmd_pub_->publish(tw);

    // Publish plan when one is available.
    publish_plan();

    const NavState cur = navigator_->state();
    if (cur != prev) {
      RCLCPP_INFO(get_logger(), "navigator_node: %s -> %s", to_string(prev),
                  to_string(cur));
    }

    // Action feedback / result.
    if (active_goal_handle_ && active_goal_handle_->is_active()) {
      auto fb = std::make_shared<NavigateToGoal::Feedback>();
      fb->state = to_string(cur);
      fb->distance_remaining = navigator_->distance_remaining(pose);
      active_goal_handle_->publish_feedback(fb);

      if (cur == NavState::SUCCEEDED || cur == NavState::FAILED) {
        auto result = std::make_shared<NavigateToGoal::Result>();
        result->success = (cur == NavState::SUCCEEDED);
        result->final_error = navigator_->distance_remaining(pose);
        result->final_state = to_string(cur);
        if (cur == NavState::SUCCEEDED) {
          active_goal_handle_->succeed(result);
        } else {
          active_goal_handle_->abort(result);
        }
        active_goal_handle_.reset();
      }
    }
  }

  void publish_plan() {
    const auto& path_opt = navigator_->path();
    if (!path_opt.has_value() || path_opt->empty()) {
      return;
    }
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = now();
    path_msg.poses.reserve(path_opt->size());
    for (const auto& wp : *path_opt) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "map";
      ps.header.stamp = path_msg.header.stamp;
      ps.pose.position.x = wp[0];
      ps.pose.position.y = wp[1];
      ps.pose.orientation.w = 1.0;
      path_msg.poses.push_back(ps);
    }
    plan_pub_->publish(path_msg);
  }

  double now_seconds() {
    const rclcpp::Time t = now();
    return static_cast<double>(t.nanoseconds()) * 1e-9;
  }

  // ----------------------------------------------------------------- members
  // Config (struct defaults are canonical; overridden by params).
  amr_core::RobotConfig robot_cfg_{};
  amr_core::NavConfig nav_cfg_{};
  amr_core::CostmapConfig costmap_cfg_{};
  amr_core::AstarConfig astar_cfg_{};
  amr_core::DwaConfig dwa_cfg_{};
  double sim_dt_{0.05};

  std::unique_ptr<Navigator> navigator_;
  std::shared_ptr<amr_planning::Costmap> costmap_;
  amr_core::Twist2D last_cmd_{0.0, 0.0};

  std::mutex mutex_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp_action::Server<NavigateToGoal>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<GoalHandle> active_goal_handle_;
};

}  // namespace amr_navigation

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_navigation::NavigatorNode>());
  rclcpp::shutdown();
  return 0;
}
