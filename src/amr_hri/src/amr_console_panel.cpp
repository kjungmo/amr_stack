// SPDX-License-Identifier: Apache-2.0
//
// AmrConsolePanel implementation.
//
// Design notes
// ============
// * The panel creates its own rclcpp::Node ("amr_console_panel") which it
//   spins via a QTimer (spin_timer_, 50 ms period = 20 Hz).  RViz2 itself
//   provides an rclcpp context; we must NOT call rclcpp::init here.
// * Goal X / Goal Y are in the "map" frame (z=0, identity orientation) per
//   CONTRACT §3 — the navigator listens to /goal_pose in map frame.
// * E-STOP: while engaged, a second QTimer (estop_timer_, 100 ms) sends
//   zero Twist on /cmd_vel so the robot stops even if the navigator is
//   still planning.  The button turns red and its label reads "RELEASE
//   E-STOP".  On release the timer stops and a final zero Twist is sent.
// * Nav-state: we register a feedback callback on the NavigateToGoal action
//   client.  The callback updates status_label_ via a thread-safe Qt signal.
//   If the action server is not available the status label reads "No action
//   server".
// * The panel stores/restores goal_x and goal_y in the rviz config.

#include "amr_hri/amr_console_panel.hpp"

#include <functional>
#include <sstream>
#include <iomanip>

#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QTimer>
#include <QVBoxLayout>
#include <QString>

#include "rviz_common/config.hpp"

namespace amr_hri {

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

AmrConsolePanel::AmrConsolePanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  setupUi();
}

AmrConsolePanel::~AmrConsolePanel()
{
  // Stop all timers before the node is destroyed.
  if (spin_timer_) {
    spin_timer_->stop();
  }
  if (estop_timer_) {
    estop_timer_->stop();
  }
}

// ---------------------------------------------------------------------------
// rviz_common::Panel interface
// ---------------------------------------------------------------------------

void AmrConsolePanel::onInitialize()
{
  // Called by RViz after the panel is fully constructed and the display
  // context is ready.  Create the ROS 2 node here (not in the ctor) so
  // rclcpp is guaranteed to be running.
  setupRos();
}

void AmrConsolePanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("goal_x", goal_x_edit_->text());
  config.mapSetValue("goal_y", goal_y_edit_->text());
}

void AmrConsolePanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString x, y;
  if (config.mapGetString("goal_x", &x)) { goal_x_edit_->setText(x); }
  if (config.mapGetString("goal_y", &y)) { goal_y_edit_->setText(y); }
}

// ---------------------------------------------------------------------------
// UI construction
// ---------------------------------------------------------------------------

void AmrConsolePanel::setupUi()
{
  auto * root = new QVBoxLayout(this);

  // --- Goal entry group -----------------------------------------------------
  auto * goal_group = new QGroupBox("Navigation Goal", this);
  auto * form = new QFormLayout();

  goal_x_edit_ = new QLineEdit("0.0", this);
  goal_y_edit_ = new QLineEdit("0.0", this);
  goal_x_edit_->setPlaceholderText("metres");
  goal_y_edit_->setPlaceholderText("metres");

  form->addRow("Goal X (m):", goal_x_edit_);
  form->addRow("Goal Y (m):", goal_y_edit_);

  send_goal_btn_ = new QPushButton("Send Goal", this);
  send_goal_btn_->setToolTip("Publish goal to /goal_pose (geometry_msgs/PoseStamped, map frame)");
  form->addRow(send_goal_btn_);

  goal_group->setLayout(form);
  root->addWidget(goal_group);

  // --- E-STOP group ---------------------------------------------------------
  auto * estop_group = new QGroupBox("Emergency Stop", this);
  auto * estop_layout = new QVBoxLayout();

  estop_btn_ = new QPushButton("E-STOP", this);
  estop_btn_->setCheckable(false);
  estop_btn_->setToolTip(
    "Engage: publishes zero /cmd_vel at 10 Hz. Release: stops publishing.");
  estop_btn_->setStyleSheet("QPushButton { background-color: #c0392b; color: white; "
                            "font-weight: bold; padding: 8px; border-radius: 4px; }");
  estop_layout->addWidget(estop_btn_);
  estop_group->setLayout(estop_layout);
  root->addWidget(estop_group);

  // --- Status group ---------------------------------------------------------
  auto * status_group = new QGroupBox("Navigator State", this);
  auto * status_layout = new QVBoxLayout();

  status_label_ = new QLabel("IDLE", this);
  status_label_->setAlignment(Qt::AlignCenter);
  status_label_->setStyleSheet("QLabel { font-size: 14pt; font-weight: bold; "
                               "padding: 6px; border: 1px solid #aaa; border-radius: 3px; }");
  status_layout->addWidget(status_label_);
  status_group->setLayout(status_layout);
  root->addWidget(status_group);

  root->addStretch();
  setLayout(root);

  // Connect buttons
  connect(send_goal_btn_, &QPushButton::clicked, this, &AmrConsolePanel::onSendGoalClicked);
  connect(estop_btn_,     &QPushButton::clicked, this, &AmrConsolePanel::onEstopClicked);
}

// ---------------------------------------------------------------------------
// ROS setup
// ---------------------------------------------------------------------------

void AmrConsolePanel::setupRos()
{
  // Create a node. RViz2 guarantees rclcpp is initialised.
  node_ = std::make_shared<rclcpp::Node>("amr_console_panel");

  // Publishers (reliable, depth 10 per CONTRACT §4)
  auto qos_reliable = rclcpp::QoS(10).reliable();
  goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", qos_reliable);
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", qos_reliable);

  // Action client for NavigateToGoal (feedback → status label)
  nav_client_ = rclcpp_action::create_client<NavAction>(node_, "navigate_to_goal");

  // QTimer: spin the node inside the Qt event loop at 20 Hz
  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, [this]() {
    rclcpp::spin_some(node_);
  });
  spin_timer_->start(50);  // 50 ms → 20 Hz

  // E-STOP timer: publishes zero Twist at 10 Hz while engaged
  estop_timer_ = new QTimer(this);
  connect(estop_timer_, &QTimer::timeout, [this]() {
    publishZeroTwist();
  });
  // Not started yet — started when E-STOP is engaged.

  // Poll for action-server availability and update status label.
  // The timer handle must be stored; a discarded rclcpp::TimerBase::SharedPtr
  // is immediately cancelled.
  server_poll_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(500),
    [this]() {
      if (!nav_client_->action_server_is_ready()) {
        QMetaObject::invokeMethod(status_label_, "setText",
          Qt::QueuedConnection,
          Q_ARG(QString, "Waiting for action server…"));
      } else {
        // Once ready, cancel this poll timer.
        server_poll_timer_->cancel();
        QMetaObject::invokeMethod(status_label_, "setText",
          Qt::QueuedConnection,
          Q_ARG(QString, "IDLE"));
      }
    });
}

// ---------------------------------------------------------------------------
// Slot: Send Goal
// ---------------------------------------------------------------------------

void AmrConsolePanel::onSendGoalClicked()
{
  bool ok_x{false}, ok_y{false};
  const double x = goal_x_edit_->text().toDouble(&ok_x);
  const double y = goal_y_edit_->text().toDouble(&ok_y);

  if (!ok_x || !ok_y) {
    status_label_->setText("Invalid X/Y — enter numbers.");
    status_label_->setStyleSheet(
      "QLabel { font-size: 14pt; font-weight: bold; padding: 6px; "
      "border: 1px solid #e74c3c; border-radius: 3px; color: #e74c3c; }");
    return;
  }

  // Restore default label style
  status_label_->setStyleSheet(
    "QLabel { font-size: 14pt; font-weight: bold; padding: 6px; "
    "border: 1px solid #aaa; border-radius: 3px; }");

  // Build PoseStamped: map frame, z=0, identity orientation (yaw=0 → w=1)
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp  = node_->get_clock()->now();
  msg.header.frame_id = "map";
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = 0.0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 1.0;

  goal_pub_->publish(msg);

  // Also send the goal to the action server so we can receive feedback.
  // If the server is not ready, skip (we still published /goal_pose).
  if (nav_client_->action_server_is_ready()) {
    NavAction::Goal action_goal;
    action_goal.goal = msg;

    auto send_opts = rclcpp_action::Client<NavAction>::SendGoalOptions{};

    // Feedback callback: update the status label on the Qt thread.
    send_opts.feedback_callback =
      [this](
        rclcpp_action::ClientGoalHandle<NavAction>::SharedPtr /*handle*/,
        const std::shared_ptr<const NavAction::Feedback> feedback)
      {
        const std::string & state = feedback->state;
        const double dist = feedback->distance_remaining;
        // Marshal to Qt thread
        QMetaObject::invokeMethod(this,
          [this, state, dist]() { updateStatusLabel(state, dist); },
          Qt::QueuedConnection);
      };

    // Result callback: update label when the action terminates.
    send_opts.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<NavAction>::WrappedResult & result)
      {
        const std::string final_state = result.result->final_state;
        QMetaObject::invokeMethod(this,
          [this, final_state]() {
            status_label_->setText(QString::fromStdString(final_state));
          },
          Qt::QueuedConnection);
      };

    nav_client_->async_send_goal(action_goal, send_opts);
  }

  status_label_->setText(
    QString("Goal sent (%1, %2)")
      .arg(x, 0, 'f', 2)
      .arg(y, 0, 'f', 2));
}

// ---------------------------------------------------------------------------
// Slot: E-STOP toggle
// ---------------------------------------------------------------------------

void AmrConsolePanel::onEstopClicked()
{
  estop_active_ = !estop_active_;

  if (estop_active_) {
    // Engage
    estop_btn_->setText("RELEASE E-STOP");
    estop_btn_->setStyleSheet(
      "QPushButton { background-color: #f39c12; color: white; "
      "font-weight: bold; padding: 8px; border-radius: 4px; }");
    status_label_->setText("E-STOP ENGAGED");
    status_label_->setStyleSheet(
      "QLabel { font-size: 14pt; font-weight: bold; padding: 6px; "
      "border: 1px solid #e74c3c; border-radius: 3px; color: #e74c3c; }");
    publishZeroTwist();
    estop_timer_->start(100);  // 10 Hz
  } else {
    // Release
    estop_timer_->stop();
    publishZeroTwist();  // one final zero command before releasing
    estop_btn_->setText("E-STOP");
    estop_btn_->setStyleSheet(
      "QPushButton { background-color: #c0392b; color: white; "
      "font-weight: bold; padding: 8px; border-radius: 4px; }");
    status_label_->setText("E-STOP released");
    status_label_->setStyleSheet(
      "QLabel { font-size: 14pt; font-weight: bold; padding: 6px; "
      "border: 1px solid #aaa; border-radius: 3px; }");
  }
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void AmrConsolePanel::publishZeroTwist()
{
  geometry_msgs::msg::Twist zero;
  // All fields default-initialised to 0.0 by ROS message definition.
  zero.linear.x  = 0.0;
  zero.linear.y  = 0.0;
  zero.linear.z  = 0.0;
  zero.angular.x = 0.0;
  zero.angular.y = 0.0;
  zero.angular.z = 0.0;
  cmd_vel_pub_->publish(zero);
}

void AmrConsolePanel::updateStatusLabel(
  const std::string & state,
  double distance_remaining)
{
  // Called from Qt thread via QMetaObject::invokeMethod.
  std::ostringstream oss;
  oss << state;
  if (distance_remaining >= 0.0) {
    oss << "  (" << std::fixed << std::setprecision(2) << distance_remaining << " m)";
  }
  status_label_->setText(QString::fromStdString(oss.str()));

  // Colour-code by state
  std::string color;
  if (state == "SUCCEEDED") {
    color = "#27ae60";
  } else if (state == "FAILED") {
    color = "#e74c3c";
  } else if (state == "RECOVERY") {
    color = "#f39c12";
  } else {
    color = "#2c3e50";
  }
  status_label_->setStyleSheet(
    QString("QLabel { font-size: 14pt; font-weight: bold; padding: 6px; "
            "border: 1px solid %1; border-radius: 3px; color: %1; }")
      .arg(QString::fromStdString(color)));
}

}  // namespace amr_hri

// Register the panel with pluginlib.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(amr_hri::AmrConsolePanel, rviz_common::Panel)
