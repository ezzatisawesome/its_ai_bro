#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),

            # RViz + map display
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [FindPackageShare("asl_tb3_sim"), "launch", "rviz.launch.py"]
                ),
                launch_arguments={
                    "config": PathJoinSubstitution(
                        [
                            FindPackageShare("autonomy_repo"),
                            "rviz",
                            "default.rviz",
                        ]
                    ),
                    "use_sim_time": use_sim_time,
                }.items(),
            ),

            # Relay RVIZ "2D Nav Goal" to /cmd_nav
            Node(
                executable="rviz_goal_relay.py",
                package="asl_tb3_lib",
                parameters=[
                    {"output_channel": "/cmd_nav"},
                ],
            ),

            # State publisher for turtlebot -> publishes /state (TurtleBotState)
            Node(
                executable="state_publisher.py",
                package="asl_tb3_lib",
                parameters=[{"use_sim_time": use_sim_time}],
            ),

            # Your navigator node
            Node(
                executable="navigator.py",
                package="autonomy_repo",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),

            # Frontier explorer node, started after a short delay
            TimerAction(
                period=3.0,   # seconds; simple and safe
                actions=[
                    Node(
                        executable="frontier_explorer_vale.py",  # your script name in autonomy_repo
                        package="autonomy_repo",
                        name="frontier_explorer",
                        output="screen",
                        parameters=[
                            {"use_sim_time": use_sim_time},
                            {"map_topic": "/map"},
                            {"state_topic": "/state"},
                            {"nav_goal_topic": "/cmd_nav"},
                            {"nav_success_topic": "/nav_success"},
                            {"window_size": 13},
                            {"unknown_pct_min": 0.20},
                            {"unoccupied_pct_min": 0.30},
                            {"occupied_zero_required": True},
                            {"plan_period": 1.0},
                            {"done_unknown_ratio": 0.05},
                        ],
                    )
                ],
            ),
        ]
    )