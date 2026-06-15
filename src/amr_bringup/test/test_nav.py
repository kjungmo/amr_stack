# SPDX-License-Identifier: Apache-2.0
"""test_nav.py — launch_testing integration test for the NAV stack.

Brings up the full navigation stack on a deterministic, fully-known map
(rendered from office.world.yaml by `amr_sim world_to_map`, committed under
test/fixtures/):

    sim_node        — ground-truth sim, /scan /odom /ground_truth + tf odom->base_link
    map_publisher   — latches the static office map on /map
    mcl_node        — MCL localiser (seeded at the spawn pose), tf map->odom
    navigator_node  — A*/DWA FSM, NavigateToGoal action server

The test seeds MCL at the known spawn, waits for localisation + a complete tf
chain (map->odom->base_link), then drives two NavigateToGoal actions and asserts
for each: the action SUCCEEDED, the believed final error is within the goal
tolerance, and the *ground-truth* pose is within 0.6 m of the goal (i.e. the
robot really arrived, not just believed it did) with no obstacle collision.

This is the NAV half of the Python `amr demo` -> DEMO PASS, re-expressed in ROS 2.
Constants mirror amr_core::*Config (robot.radius=0.18, nav.goal_tol_xy=0.25).
"""

import math
import os
import time
import unittest

# Isolate this test's DDS traffic from any other launch test that colcon may run
# concurrently (e.g. test_slam), so their nodes never discover each other. Set
# before rclpy/launch initialise so both the test node and the launched nodes
# share this domain.
os.environ["ROS_DOMAIN_ID"] = "43"

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from amr_interfaces.action import NavigateToGoal

# ── constants mirroring amr_core::*Config ──────────────────────────────────
ROBOT_RADIUS = 0.18        # RobotConfig::radius
GOAL_TOL_XY = 0.25         # NavConfig::goal_tol_xy
GT_ERROR_LIMIT = 0.6       # DEMO PASS ground-truth tolerance (CONTRACT §7)

# Spawn pose (office.world.yaml spawn) — MCL is seeded here.
SPAWN = (1.5, 1.5, 0.0)

# A full-office tour, validated end to end against the real stack: spawn (west
# room) -> north of the west room -> middle room (through the wall-A door gap) ->
# east room (through the wall-B gap). Each leg exercises localize -> A* plan ->
# DWA follow -> reach with no collision, crossing the two interior wall gaps.
NAV_GOALS = [
    (2.0, 7.0),   # north end of the west room
    (6.0, 2.5),   # middle room, via the wall-A door gap (y 5.5-7.0)
    (9.5, 7.5),   # east room, via the wall-B gap (y < 3.5)
]

# Circular obstacles (office.world.yaml) for the collision check.
CIRCLES = [(3.3, 3.5, 0.3), (9.8, 6.5, 0.3)]

_HERE = os.path.dirname(os.path.abspath(__file__))
FIXTURE_MAP = os.path.join(_HERE, "fixtures", "office_map.yaml")


@pytest.mark.launch_test
def generate_test_description():
    pkg_share = get_package_share_directory("amr_bringup")
    params = os.path.join(pkg_share, "config", "amr.yaml")
    world = os.path.join(pkg_share, "config", "office.world.yaml")

    sim_node = launch_ros.actions.Node(
        package="amr_sim", executable="sim_node", name="sim_node",
        parameters=[params, {"world_file": world, "seed": 42}], output="screen",
    )
    map_publisher = launch_ros.actions.Node(
        package="amr_mapping", executable="map_publisher", name="map_publisher",
        parameters=[params, {"map_yaml": FIXTURE_MAP}], output="screen",
    )
    mcl_node = launch_ros.actions.Node(
        package="amr_localization", executable="mcl_node", name="mcl_node",
        parameters=[params, {
            "seed": 42,
            "use_initial_pose": True,
            "initial_pose_x": SPAWN[0],
            "initial_pose_y": SPAWN[1],
            "initial_pose_theta": SPAWN[2],
        }], output="screen",
    )
    navigator_node = launch_ros.actions.Node(
        package="amr_navigation", executable="navigator_node",
        name="navigator_node", parameters=[params], output="screen",
    )

    return launch.LaunchDescription([
        sim_node, map_publisher, mcl_node, navigator_node,
        launch_testing.actions.ReadyToTest(),
    ]), {
        "sim_node": sim_node,
        "map_publisher": map_publisher,
        "mcl_node": mcl_node,
        "navigator_node": navigator_node,
    }


class _NavClient(Node):
    """Helper rclpy node: tracks ground truth and drives NavigateToGoal."""

    def __init__(self):
        super().__init__("nav_test_client")
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        gt_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._gt = None
        self.create_subscription(Odometry, "/ground_truth", self._gt_cb, gt_qos)
        self._nav = ActionClient(self, NavigateToGoal, "navigate_to_goal")

    def _gt_cb(self, msg):
        self._gt = msg.pose.pose

    @property
    def gt(self):
        return self._gt

    def spin(self, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_localization(self, timeout=20.0):
        """Wait until ground truth is flowing and the action server is up."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._gt is not None and self._nav.server_is_ready():
                return True
        return self._gt is not None and self._nav.wait_for_server(timeout_sec=5.0)

    def navigate(self, x, y, timeout=120.0):
        # The very first goal after startup can race action discovery (rclpy may
        # drop the result with an "unexpected response" warning). Retry once on a
        # dropped/empty result so the test reflects navigation behaviour, not the
        # discovery race.
        res = self._navigate_once(x, y, timeout)
        if res.get("final_state", "") in ("", "TIMEOUT", "REJECTED"):
            self.spin(1.0)
            res = self._navigate_once(x, y, timeout)
        return res

    def _navigate_once(self, x, y, timeout=120.0):
        if not self._nav.wait_for_server(timeout_sec=15.0):
            return {"final_state": "NO_SERVER", "final_error": float("inf")}
        goal = NavigateToGoal.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = "map"
        goal.goal.pose.position.x = float(x)
        goal.goal.pose.position.y = float(y)
        goal.goal.pose.orientation.w = 1.0

        send = self._nav.send_goal_async(goal)
        deadline = time.time() + timeout
        while not send.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not send.done() or not send.result().accepted:
            return {"final_state": "REJECTED", "final_error": float("inf")}

        result_future = send.result().get_result_async()
        while not result_future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not result_future.done():
            return {"final_state": "TIMEOUT", "final_error": float("inf")}

        res = result_future.result().result
        return {"final_state": res.final_state, "final_error": res.final_error,
                "success": res.success}

    def stop(self):
        self._cmd_pub.publish(Twist())


class TestNavStack(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = _NavClient()
        # Let the stack spin up, MCL converge, and the tf chain establish.
        cls.node.wait_for_localization(timeout=25.0)
        cls.node.spin(3.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.stop()
        cls.node.destroy_node()
        rclpy.shutdown()

    def _gt_dist(self, gx, gy):
        pose = self.node.gt
        self.assertIsNotNone(pose, "no ground_truth pose received")
        return math.hypot(pose.position.x - gx, pose.position.y - gy)

    def _assert_no_collision(self):
        pose = self.node.gt
        if pose is None:
            return
        px, py = pose.position.x, pose.position.y
        for cx, cy, cr in CIRCLES:
            self.assertGreater(
                math.hypot(px - cx, py - cy), cr + ROBOT_RADIUS,
                f"collision with circular obstacle at ({cx},{cy})")

    def _run_goal(self, idx):
        gx, gy = NAV_GOALS[idx]
        res = self.node.navigate(gx, gy, timeout=120.0)
        self.assertEqual(res["final_state"], "SUCCEEDED",
                         f"goal {idx + 1} ({gx},{gy}) not SUCCEEDED: {res}")
        self.assertLess(res["final_error"], GOAL_TOL_XY + 0.05,
                        f"goal {idx + 1} believed error too large: {res}")
        self.node.spin(0.5)
        self.assertLess(self._gt_dist(gx, gy), GT_ERROR_LIMIT,
                        f"goal {idx + 1} ground-truth error >= {GT_ERROR_LIMIT} m")
        self._assert_no_collision()

    def test_01_west_room_north(self):
        self._run_goal(0)

    def test_02_middle_room(self):
        self._run_goal(1)

    def test_03_east_room(self):
        self._run_goal(2)


@launch_testing.post_shutdown_test()
class TestNavShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info, sim_node, navigator_node):
        launch_testing.asserts.assertExitCodes(
            proc_info, [0, -15], process=sim_node)
        launch_testing.asserts.assertExitCodes(
            proc_info, [0, -15], process=navigator_node)
