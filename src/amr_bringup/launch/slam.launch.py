# SPDX-License-Identifier: Apache-2.0
"""slam.launch.py — Start sim_node + slam_node + rviz2 for SLAM mode.

Nodes:
  sim_node       (amr_sim)    — publishes /scan, /odom, /ground_truth, tf
  slam_node      (amr_slam)   — publishes /map, tf map->odom; serves /save_map
  rviz2                       — visualisation with amr.rviz

Launch arguments:
  world_file    path to world YAML         (default: office.world.yaml in config/)
  params_file   path to ROS params YAML    (default: amr.yaml in config/)
  rviz          launch rviz2? [true/false] (default: true)
  seed          RNG seed                   (default: "42")
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("amr_bringup")
    default_world = str(Path(pkg_share) / "config" / "office.world.yaml")
    default_params = str(Path(pkg_share) / "config" / "amr.yaml")
    default_rviz = str(Path(pkg_share) / "rviz" / "amr.rviz")

    # ── declare arguments ────────────────────────────────────────────────────
    arg_world = DeclareLaunchArgument(
        "world_file",
        default_value=default_world,
        description="Path to world YAML file",
    )
    arg_params = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to ROS 2 params YAML file",
    )
    arg_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz2 (true/false)",
    )
    arg_seed = DeclareLaunchArgument(
        "seed",
        default_value="42",
        description="Global RNG seed",
    )

    params_file = LaunchConfiguration("params_file")
    world_file = LaunchConfiguration("world_file")
    seed = LaunchConfiguration("seed")

    # ── sim_node ─────────────────────────────────────────────────────────────
    sim_node = Node(
        package="amr_sim",
        executable="sim_node",
        name="sim_node",
        output="screen",
        parameters=[
            params_file,
            {
                "world_file": world_file,
                "seed": seed,
            },
        ],
    )

    # ── slam_node ─────────────────────────────────────────────────────────────
    slam_node = Node(
        package="amr_slam",
        executable="slam_node",
        name="slam_node",
        output="screen",
        parameters=[
            params_file,
            {"seed": seed},
        ],
    )

    # ── rviz2 ─────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", default_rviz],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            arg_world,
            arg_params,
            arg_rviz,
            arg_seed,
            sim_node,
            slam_node,
            rviz_node,
        ]
    )
