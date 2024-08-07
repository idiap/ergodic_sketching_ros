# SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
#
# SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
#
# SPDX-License-Identifier: GPL-3.0-only

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "config_file",
            default_value="drozbot_config_iiwa.yaml",
            description="Configuration file for ergodic sketcher",
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="world",
            description="Base frame",
        ),
        DeclareLaunchArgument(
            "use_gui",
            default_value="true",
            description="Display UI",
        ),
        DeclareLaunchArgument(
            "drawing_frame_xyz",
            default_value="0.6 0.15 0.2",
            description="Drawing frame position",
        ),
        DeclareLaunchArgument(
            "drawing_frame_rpy",
            default_value="0 0 -1.5708",
            description="Drawing frame orientation",
        )
    ]

    config_file = LaunchConfiguration("config_file")
    base_frame = LaunchConfiguration("base_frame")
    use_gui = LaunchConfiguration("use_gui")
    drawing_frame_xyz = LaunchConfiguration("drawing_frame_xyz")
    drawing_frame_rpy = LaunchConfiguration("drawing_frame_rpy")

    cwd = FindPackageShare("ergodic_sketching_ros")

    planner_config_file = PathJoinSubstitution(
        [
            FindPackageShare("ergodic_sketching_ros"),
            "config",
            "ilqr_planner_config_iiwa.yaml"
        ]
    )
    
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("ergodic_sketching_ros"),
            "rviz",
            "config.rviz"
        ]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ergodic_sketching_ros"),
                    "descriptions",
                    "iiwa_drawing.urdf.xacro"
                ]
            ),
            " ",
            "drawing_frame_xyz:=",
            "'",
            drawing_frame_xyz,
            "'",
            " ",
            "drawing_frame_rpy:=",
            "'",
            drawing_frame_rpy,
            "'",
        ]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content,value_type=str)}

    ergodic_sketching_node = Node(
        package="ergodic_sketching_ros",
        executable="ergodic_sketching_ros",
        name="ergodic_sketching_ros",
        parameters=[
            robot_description,
            {
                "path":cwd,
                "config_file":config_file,
                "base_frame":base_frame,
                "drawing_frame_rpy":drawing_frame_rpy,
                "drawing_frame_xyz":drawing_frame_xyz,
            },
        ],
        output="both",
    )
    ilqr_planner_node = Node(
        package="ergodic_sketching_ros",
        executable="planner_action_server",
        name="planner_action_server",
        parameters=[
            robot_description,
            planner_config_file,
            {
                "drawing_frame_rpy":drawing_frame_rpy,
                "drawing_frame_xyz":drawing_frame_xyz,

            }
        ],
        output="both",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "--display-config",
            rviz_config_file,
            "-f",
            "world"
        ],
        output="both",
        condition=IfCondition(use_gui),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            robot_description
        ],
        output="both",
        condition=IfCondition(use_gui),
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[
            {"source_list":["rob_sim/joint_states"]}
        ],
        output="both",
        condition=IfCondition(use_gui),
    )

    nodes = [
        ergodic_sketching_node,
        ilqr_planner_node,
        rviz_node,
        robot_state_publisher_node,
        joint_state_publisher_node
    ]

    return LaunchDescription(arguments + nodes)