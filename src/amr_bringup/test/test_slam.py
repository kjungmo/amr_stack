# SPDX-License-Identifier: Apache-2.0
"""test_slam.py — launch_testing integration test for the SLAM stack.

Brings up sim_node + slam_node, performs a bounded mapping pass around the
spawn (a rotate-in-place sweep plus a few short, collision-free moves in the
open west room), then calls /save_map and asserts a genuine map was built:
the saved ROS map_server PGM exists and contains a meaningful number of both
occupied (wall) and free (interior) cells.

This is the SLAM half of the Python `amr demo` -> DEMO PASS, re-expressed in
ROS 2. The driver deliberately uses short, open moves (no obstacle avoidance is
under test here) and skips any move it cannot complete, so the test exercises
SLAM map construction rather than a navigation controller.
"""

import math
import os
import time
import unittest

# Isolate this test's DDS traffic from any other launch test that colcon may run
# concurrently (e.g. test_nav), so their nodes never discover each other. Set
# before rclpy/launch initialise so both the test node and the launched nodes
# share this domain.
os.environ["ROS_DOMAIN_ID"] = "42"

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from amr_interfaces.srv import SaveMap

_MAP_STEM = os.path.join("/tmp", "amr_test_slam_map")

# Short, collision-free moves in the open west room (west of wall A at x=4,
# clear of the pillar at (3.3, 3.5)). Each is optional — skipped if not reached.
PATROL = [(1.2, 3.0), (1.2, 5.0), (2.5, 5.0), (2.5, 2.5), (1.5, 1.5)]


@pytest.mark.launch_test
def generate_test_description():
    pkg_share = get_package_share_directory("amr_bringup")
    params = os.path.join(pkg_share, "config", "amr.yaml")
    world = os.path.join(pkg_share, "config", "office.world.yaml")

    sim_node = launch_ros.actions.Node(
        package="amr_sim", executable="sim_node", name="sim_node",
        parameters=[params, {"world_file": world, "seed": 42}], output="screen",
    )
    slam_node = launch_ros.actions.Node(
        package="amr_slam", executable="slam_node", name="slam_node",
        parameters=[params, {"seed": 42}], output="screen",
    )
    return launch.LaunchDescription([
        sim_node, slam_node, launch_testing.actions.ReadyToTest(),
    ]), {"sim_node": sim_node, "slam_node": slam_node}


class _Driver(Node):
    def __init__(self):
        super().__init__("slam_test_driver")
        self._cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        gt_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE)
        self._gt = None
        self.create_subscription(Odometry, "/ground_truth", self._gt_cb, gt_qos)
        self._save = self.create_client(SaveMap, "/save_map")

    def _gt_cb(self, m):
        self._gt = m.pose.pose

    def spin(self, secs):
        dl = time.time() + secs
        while time.time() < dl:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_gt(self, timeout=15.0):
        dl = time.time() + timeout
        while self._gt is None and time.time() < dl:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._gt is not None

    def rotate(self, secs=14.0, wz=0.6):
        """Rotate in place to sweep the lidar around the spawn."""
        dl = time.time() + secs
        tw = Twist()
        tw.angular.z = wz
        while time.time() < dl:
            self._cmd.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.stop()

    def drive_to(self, wx, wy, timeout=22.0, tol=0.35):
        dl = time.time() + timeout
        while time.time() < dl:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._gt is None:
                continue
            px, py = self._gt.position.x, self._gt.position.y
            d = math.hypot(wx - px, wy - py)
            if d < tol:
                self.stop()
                return True
            q = self._gt.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            herr = math.atan2(wy - py, wx - px) - yaw
            herr = (herr + math.pi) % (2 * math.pi) - math.pi
            tw = Twist()
            tw.linear.x = min(0.35, 0.5 * d) if abs(herr) < 0.6 else 0.05
            tw.angular.z = max(-1.2, min(1.2, 1.8 * herr))
            self._cmd.publish(tw)
        self.stop()
        return False

    def stop(self):
        self._cmd.publish(Twist())

    def save_map(self, stem, timeout=15.0):
        if not self._save.wait_for_service(timeout_sec=timeout):
            return False
        req = SaveMap.Request()
        req.path = stem
        fut = self._save.call_async(req)
        dl = time.time() + timeout
        while not fut.done() and time.time() < dl:
            rclpy.spin_once(self, timeout_sec=0.1)
        return fut.done() and fut.result().success


def _read_pgm_counts(pgm_path):
    with open(pgm_path, "rb") as f:
        data = f.read()
    i = 0

    def tok():
        nonlocal i
        while data[i] in b" \t\r\n":
            i += 1
        s = i
        while data[i] not in b" \t\r\n":
            i += 1
        return data[s:i]

    assert tok() == b"P5"
    cols = int(tok())
    rows = int(tok())
    int(tok())
    i += 1
    px = data[i:i + cols * rows]
    occ = sum(1 for v in px if v <= 50)
    free = sum(1 for v in px if v >= 250)
    return cols, rows, occ, free


class TestSlam(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = _Driver()
        cls.node.wait_gt(timeout=15.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.stop()
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_01_map_a_region(self):
        self.assertTrue(self.node.wait_gt(5.0), "no ground_truth from sim")
        # Sweep, then patrol the open west room (skipping unreachable moves).
        self.node.rotate(secs=14.0, wz=0.6)
        for wx, wy in PATROL:
            self.node.drive_to(wx, wy, timeout=22.0, tol=0.35)
            self.node.rotate(secs=4.0, wz=0.8)
        self.node.stop()
        self.node.spin(1.0)

    def test_02_save_and_validate(self):
        ok = self.node.save_map(_MAP_STEM, timeout=15.0)
        self.assertTrue(ok, "/save_map returned success=false or unavailable")
        pgm = _MAP_STEM + ".pgm"
        yml = _MAP_STEM + ".yaml"
        self.assertTrue(os.path.isfile(pgm), f"map PGM not written: {pgm}")
        self.assertTrue(os.path.isfile(yml), f"map YAML not written: {yml}")
        cols, rows, occ, free = _read_pgm_counts(pgm)
        # A genuine map of the west room has substantial walls and interior.
        self.assertGreater(occ, 150, f"too few occupied cells ({occ}) — no walls mapped")
        self.assertGreater(free, 1500, f"too few free cells ({free}) — no interior mapped")


@launch_testing.post_shutdown_test()
class TestSlamShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info, sim_node, slam_node):
        launch_testing.asserts.assertExitCodes(proc_info, [0, -15], process=sim_node)
        launch_testing.asserts.assertExitCodes(proc_info, [0, -15], process=slam_node)
