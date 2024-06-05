"""
A launch file for running the motion planning python api tutorial
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path



def generate_launch_description():

    is_simulation = os.getenv('USE_SIM')
    if is_simulation == 'False': is_simulation = False
    else: is_simulation = True

    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        output="screen"
    )

    static_tf_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["--frame-id", "kbase_link", "--child-frame-id", "camera_link"],
        parameters = [
            {"use_sim_time": is_simulation}
        ]
    )

    return LaunchDescription(
        [
            realsense_node,
            static_tf_camera
        ]
    )