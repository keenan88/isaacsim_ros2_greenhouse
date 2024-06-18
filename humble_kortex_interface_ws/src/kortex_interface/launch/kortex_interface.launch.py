from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
import os




def generate_launch_description():

    gen3_sim_args = {
        'robot_ip': 'yyy.yyy.yyy.yyy',
        'use_fake_hardware': "true"
        'dof': '6',
        'gripper' : '""',
        # 'launch_rviz': 'false',
    }.items()

    gen3_hw_args = {
        'robot_ip': '192.168.1.10',
        'use_fake_hardware': "false"
        'dof': '6',
        'gripper' : '""',
        # 'launch_rviz': 'false',
    }.items()

    gen3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('kortex_bringup'),
                'launch',
                'gen3.launch.py'
            ),
        ),
        launch_arguments=gen3_sim_args,
    )

    # joint_pruner = Node(
    #     package="kortex_interface",
    #     executable = "joint_pruner",
    #     name = "joint_pruner"
    # )

    # domain_bridge_config_path = os.path.join(
    #     get_package_share_directory('kortex_interface'), 
    #     'config', 
    #     'kortex_domain_bridge.yaml'
    # )

    # domain_bridge = Node(
    #     package="domain_bridge",
    #     executable="domain_bridge",
    #     name = "domain_bridge",
    #     arguments = [domain_bridge_config_path]
    # )

    joint_command_forwarder = Node(
        package = "kortex_interface",
        executable = "joint_command_forwarder",
        output = "screen"
    )

    sim_launch = [
        # PushRosNamespace(namespace)
        gen3_bringup,
        joint_command_forwarder
    ]

    hw_launch = [
        # PushRosNamespace(namespace)
        gen3_bringup
    ]

    return LaunchDescription(
        sim_launch
    )
