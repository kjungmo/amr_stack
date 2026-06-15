# SPDX-License-Identifier: Apache-2.0
"""nav.launch.py — Start sim_node + map_publisher + mcl_node + navigator_node + rviz2.

Nodes:
  sim_node          (amr_sim)          — publishes /scan, /odom, /ground_truth, tf
  map_publisher     (amr_mapping)      — latches a pre-built /map (TransientLocal)
  mcl_node          (amr_localization) — MCL localiser, publishes /pose, /particles
  navigator_node    (amr_navigation)   — action server NavigateToGoal, FSM
  rviz2                                — visualisation with amr.rviz

Launch arguments:
  map_file      path to map YAML (required — output of /save_map)
  world_file    path to world YAML (default: office.world.yaml in config/)
  params_file   path to ROS params YAML (default: amr.yaml in config/)
  rviz          launch rviz2? [true/false] (default: true)
  seed          RNG seed (default: "42")
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("amr_bringup")
    default_world = str(Path(pkg_share) / "config" / "office.world.yaml")
    default_params = str(Path(pkg_share) / "config" / "amr.yaml")
    default_rviz = str(Path(pkg_share) / "rviz" / "amr.rviz")

    # ── declare arguments ────────────────────────────────────────────────────
    arg_map = DeclareLaunchArgument(
        "map_file",
        default_value="",
        description="Path to pre-built map YAML (stem.yaml produced by /save_map)",
    )
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
    map_file = LaunchConfiguration("map_file")
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

    # ── map_publisher ─────────────────────────────────────────────────────────
    map_publisher_node = Node(
        package="amr_mapping",
        executable="map_publisher",
        name="map_publisher",
        output="screen",
        parameters=[
            params_file,
            {"map_yaml": map_file},
        ],
    )

    # ── mcl_node ──────────────────────────────────────────────────────────────
    mcl_node = Node(
        package="amr_localization",
        executable="mcl_node",
        name="mcl_node",
        output="screen",
        parameters=[
            params_file,
            {"seed": seed},
        ],
    )

    # ── navigator_node ────────────────────────────────────────────────────────
    navigator_node = Node(
        package="amr_navigation",
        executable="navigator_node",
        name="navigator_node",
        output="screen",
        parameters=[params_file],
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
            arg_map,
            arg_world,
            arg_params,
            arg_rviz,
            arg_seed,
            sim_node,
            map_publisher_node,
            mcl_node,
            navigator_node,
            rviz_node,
        ]
    )
