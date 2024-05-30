from launch import LaunchDescription
from launch_ros.actions import Node
import os 

def generate_launch_description():

    gen3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('kortex_bringup'),
                'launch',
                'gen3.launch.py'
            )
        ),
        launch_arguments={
            'robot_ip': '192.168.1.10',
            'dof': '6',
            'launch_rviz': 'false'
        }.items()
    )

    return LaunchDescription([
        gen3_bringup
    ])
