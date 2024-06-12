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
            os.path.join(get_package_share_directory('realsense2_camera'), 'examples', 'pointcloud'),
            '/rs_d455_pointcloud_launch.py'
        ])
    )


    arm_base_to_camera_static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='arm_base_to_camera_static_tf',
            output='screen',
            arguments=[
                 '--x', '0.0', 
                 '--y', '0.0', 
                 '--z', '1.0',       # translation (x, y, z)
                 '--qx', '0.0', 
                 '--qy', '0.0', 
                 '--qz', '0.0', 
                 '--qw', '1.0', # rotation (quaternion: x, y, z, w)
                '--frame-id', 'camera_link',                # parent frame
                '--child-frame-id', 'arm_base_link'                     # child frame
            ]
        )

    

    return LaunchDescription(
        [
            realsense_pointcloud_launch,
            arm_base_to_camera_static_tf
        ]    
    )