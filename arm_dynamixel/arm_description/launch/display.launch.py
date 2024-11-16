from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import xacro
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    declared_arguments = []

    arm_description_path = os.path.join(
        get_package_share_directory('arm_description'))

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file", #this will be the name of the argument  
            default_value=PathJoinSubstitution(
                [FindPackageShare("arm_description"), "config", "rviz", "standing.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz.",
        )
    )

    
    urdf_arm = os.path.join(arm_description_path, "urdf", "arm.urdf.xacro")

    rviz_config = os.path.join(arm_description_path, "config", "rviz", "standing.rviz")

    with open(urdf_arm, 'r') as infp:
        arm_desc = infp.read()


    robot_description_arm_xacro = {"robot_description": Command(['xacro ', urdf_arm])}  

    robot_description_arm_urdf = {"robot_description": arm_desc}  

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description_arm_urdf,
                    {"use_sim_time": True},
            ],
    )
 
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
    )


    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,  
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start) 
