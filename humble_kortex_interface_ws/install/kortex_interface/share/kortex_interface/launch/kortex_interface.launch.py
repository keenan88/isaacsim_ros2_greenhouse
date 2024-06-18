from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
import os




def generate_launch_description():

    is_simulation = os.getenv('USE_SIM')
    if is_simulation == 'False': is_simulation = False
    else: is_simulation = True

    gen3_sim_args = {
        'robot_ip': 'yyy.yyy.yyy.yyy',
        'use_fake_hardware': "true",
        'dof': '6',
        'gripper' : '""',
        # 'launch_rviz': 'false',
    }.items()

    gen3_hw_args = {
        'robot_ip': '192.168.1.10',
        'use_fake_hardware': "false",
        'dof': '6',
        'gripper' : '""',
        # 'launch_rviz': 'false',
    }.items()

    gen3_args = gen3_sim_args if is_simulation else gen3_hw_args

    gen3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('kortex_bringup'),
                'launch',
                'gen3.launch.py'
            ),
        ),
        launch_arguments = gen3_args,
    )

    joint_command_forwarder = Node(
        package = "kortex_interface",
        executable = "joint_command_forwarder",
        output = "screen"
    )

    namespace = 'kortex_interface'

    sim_launch = [
        PushRosNamespace(namespace),
        gen3_bringup,
        joint_command_forwarder
    ]

    hw_launch = [
        #PushRosNamespace(namespace),
        gen3_bringup
    ]

    gen3_launch = sim_launch if is_simulation else hw_launch

    return LaunchDescription(
        hw_launch
    )
