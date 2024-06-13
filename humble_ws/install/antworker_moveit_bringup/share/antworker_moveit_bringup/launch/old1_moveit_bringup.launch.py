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
import yaml
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    is_simulation = os.getenv('USE_SIM')
    if is_simulation == 'False': is_simulation = False
    else: is_simulation = True

    
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="antworker", 
            package_name="antworker_moveit_description",
        )
        .robot_description(
            file_path = "config/antworker.urdf.xacro"
        )
        .trajectory_execution(file_path = "config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path = get_package_share_directory("antworker_moveit_bringup")
            + "/config/moveit_py_params.yaml"
        )
        .planning_pipelines(
            pipelines = ["ompl"] # "chomp", "pilz_industrial_motion_planner", "stomp"
        )
        .planning_scene_monitor(
            publish_robot_description = False, publish_robot_description_semantic = True
        )
        .robot_description_semantic(file_path = "config/antworker.srdf")
        .sensors_3d(
            file_path = "config/sensors_3d.yaml"
        )
        .to_moveit_configs()
    )

    octomap_config = {
        'octomap_frame': 'sim_camera', 
        'octomap_resolution': 0.05,
        'max_range': 5.0
    }

    rviz_config_file = os.path.join(
        "/home/humble_ws/src/antworker_moveit_bringup/config",
        "rviz_config.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {"use_sim_time": is_simulation}
        ],
    )

    base_to_arm_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["--frame-id", "kbase_link", "--child-frame-id", "arm_base_link"],
        parameters = [
            {"use_sim_time": is_simulation}
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[
            moveit_config.robot_description,
            {
                "use_sim_time": is_simulation
            }
        ],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("antworker_moveit_description"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ros2_controllers_path,
            {"use_sim_time": is_simulation}
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    spawn_joint_trajectory_controller = ExecuteProcess(
        cmd=["ros2 run controller_manager spawner {}".format("kinova_arm_controller")],
        shell=True,
        output="screen"
    )

    spawn_joint_state_brodcaster = ExecuteProcess(
        cmd=["ros2 run controller_manager spawner {}".format("joint_state_broadcaster")],
        shell=True,
        output="screen"
    )
    
    joint_command_forwarder = Node(
        package="antworker_moveit_bringup",
        executable="joint_command_forwarder",
        output="log",
        parameters = [
            {"use_sim_time": is_simulation}
        ]
    )

    hardware_launch_description = [
        ee_point_server, 
        base_to_arm_static_tf,
        rviz_node,
        robot_state_publisher,
    ]

    simulation_launch = [
        ee_point_server,
        robot_state_publisher,
        ros2_controller_manager,
        spawn_joint_trajectory_controller,
        spawn_joint_state_brodcaster,
        joint_command_forwarder,
        rviz_node
    ]



    launch_content = simulation_launch if is_simulation else hardware_launch_description

    return LaunchDescription(
        launch_content
    )