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

    
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="antworker", 
            package_name="antworker_moveit_description",
        )
        .robot_description(
            file_path="config/antworker.urdf.xacro"
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("antworker_moveit_bringup")
            + "/config/moveit_py_params.yaml"
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"]
        )
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .robot_description_semantic(file_path="config/antworker.srdf")
        .to_moveit_configs()
    )

    ee_point_server = Node(
        name="ee_point_server",
        package="antworker_moveit_bringup",
        executable="ee_point_server",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": is_simulation}
        ],
    )

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

    urdf_file_path = Path("/home/humble_ws/src/antworker_description/description/combined/worker_with_arm.urdf")

    robot_description_content = urdf_file_path.read_text()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {
                "use_sim_time": is_simulation,
            #    "robot_description": robot_description_content
            }
        ],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("antworker_moveit_description"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
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

    load_controllers = []
    for controller in [
        "kinova_arm_controller",
        # "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen"
            )
        ]

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": is_simulation}
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    trajectory_server_hardware = Node(
        package="antworker_moveit_bringup",
        executable="trajectory_server",
        output="screen",
        parameters = [
            {"use_sim_time": is_simulation}
        ]
    )

    trajectory_server_sim = Node(
        package="antworker_moveit_bringup",
        executable="trajectory_server_sim",
        output="screen",
        parameters = [
            {"use_sim_time": is_simulation}
        ]
    )

    # realsense_node = Node(
    #     package="realsense2_camera",
    #     executable="realsense2_camera_node",
    #     output="screen"
    # )

    # static_tf_camera = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="screen",
    #     arguments=["--frame-id", "kbase_link", "--child-frame-id", "camera_link"],
    #     parameters = [
    #         {"use_sim_time": is_simulation}
    #     ]
    # )

    hardware_launch_description = [
        ee_point_server, 
        move_group, 
        trajectory_server_hardware, 
        base_to_arm_static_tf,
        rviz_node,
        robot_state_publisher
    ]

    simulation_launch = [
        ee_point_server,
        move_group,
        trajectory_server_sim,
        ros2_control_node,
        robot_state_publisher,
        rviz_node
    ] + load_controllers



    launch_content = simulation_launch if is_simulation else hardware_launch_description

    return LaunchDescription(
        launch_content
    )