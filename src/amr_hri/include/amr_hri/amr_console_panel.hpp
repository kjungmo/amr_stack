// SPDX-License-Identifier: Apache-2.0
//
// amr_hri — AmrConsolePanel
// RViz2 panel plugin (rviz_common::Panel subclass) providing:
//   - Goal X / Goal Y entry + "Send Goal" button → publishes /goal_pose
//   - E-STOP toggle button → publishes zero /cmd_vel while engaged
//   - Status label → subscribes to NavigateToGoal action feedback
//
// Nav-state source: amr_interfaces::action::NavigateToGoal feedback field
// `state` (string: IDLE|PLANNING|FOLLOWING|RECOVERY|SUCCEEDED|FAILED).
// Chosen over a /nav_state std_msgs/String topic so the panel is consistent
// with how amr_navigation's action server already exposes state; no extra
// publisher node is required.
#pragma once

#include <memory>
#include <string>

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QWidget>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "amr_interfaces/action/navigate_to_goal.hpp"
#include "rviz_common/panel.hpp"

namespace amr_hri {

class AmrConsolePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit AmrConsolePanel(QWidget * parent = nullptr);
  ~AmrConsolePanel() override;

  // rviz_common::Panel interface
  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void onSendGoalClicked();
  void onEstopClicked();

private:
  // ---- ROS handles ----------------------------------------------------------
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_vel_pub_;

  /// Action client to NavigateToGoal — used only for subscribing to feedback.
  /// We do NOT send a goal from here (goal is sent via /goal_pose topic per
  /// CONTRACT §4); we merely observe existing goals' feedback.
  using NavAction   = amr_interfaces::action::NavigateToGoal;
  using NavClient   = rclcpp_action::Client<NavAction>;
  NavClient::SharedPtr nav_client_;

  /// Spinning timer — drives rclcpp::spin_some so the node processes callbacks
  /// inside the Qt event loop without a separate thread.
  QTimer * spin_timer_{nullptr};

  /// Periodic wall timer used only to poll for action-server availability
  /// and update the status label.  Must be stored to prevent early cancellation.
  rclcpp::TimerBase::SharedPtr server_poll_timer_;

  // ---- E-STOP state ---------------------------------------------------------
  bool estop_active_{false};

  /// Publishes zero Twist on /cmd_vel every 100 ms while E-STOP is engaged.
  QTimer * estop_timer_{nullptr};

  // ---- Qt widgets -----------------------------------------------------------
  QLineEdit * goal_x_edit_{nullptr};
  QLineEdit * goal_y_edit_{nullptr};
  QPushButton * send_goal_btn_{nullptr};
  QPushButton * estop_btn_{nullptr};
  QLabel * status_label_{nullptr};

  // ---- helpers --------------------------------------------------------------
  void updateStatusLabel(const std::string & state, double distance_remaining);
  void publishZeroTwist();
  void setupRos();
  void setupUi();
};

}  // namespace amr_hri
