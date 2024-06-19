import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    is_simulation = os.getenv('USE_SIM')
    if is_simulation == 'False': is_simulation = False
    else: is_simulation = True

    realsense_pointcloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ]),
        launch_arguments = {
            'camera_name' :  "camera",
            'rgb_camera.color_profile': '424x240x5',
            'depth_module.depth_profile': '424x240x5',
            'depth_module.infra_profile': '424x240x5',
            'depth_module.color_profile': '480x270x6',
            'pointcloud.enable': 'true'

        }.items()
    )

    tf_replace_base_link = Node(
        package = "realsense_bringup",
        executable = "tf_filter",
        output = "screen"
    )

    domain_bridge_config_path = os.path.join(
        get_package_share_directory('realsense_bringup'), 
        'config', 
        'realsense_domain_bridge.yaml'
    )

    domain_bridge = Node(
        package="domain_bridge",
        executable="domain_bridge",
        name = "domain_bridge",
        arguments = [domain_bridge_config_path]
    )

    rviz2_node = Node(
        package = "rviz2",
        executable = "rviz2",
        output = 'screen',
        arguments = ['-d', '/home/humble_ws/src/realsense_bringup/config/realsense_viewer.rviz']
    )

    

    return LaunchDescription(
        [
            realsense_pointcloud_launch,
            tf_replace_base_link,
            domain_bridge,
            # rviz2_node
        ]    
    )