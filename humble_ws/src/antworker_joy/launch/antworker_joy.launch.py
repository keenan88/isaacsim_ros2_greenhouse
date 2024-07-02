import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    is_simulation = os.getenv('USE_SIM')
    if is_simulation == 'False': is_simulation = False
    else: is_simulation = True

    joy_teleop_launch_file = os.path.join(
        get_package_share_directory('teleop_twist_joy'),
        'launch',
        'teleop-launch.py'
    )

    joy_driver_and_cmd_vel_provider = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joy_teleop_launch_file),
        launch_arguments={
            # TODO - convert joy config filepath to a docker container path, once figured out how to stop xbox controller from timing out
            'config_filepath': '/home/keenan/Downloads/isaacsim_ros2_greenhouse/humble_ws/src/antworker_joy/config/joy_config.yaml', 
            'joy_config' : 'xbox'
        }.items()
    )





    return LaunchDescription(
        [
            joy_driver_and_cmd_vel_provider
        ]
    )