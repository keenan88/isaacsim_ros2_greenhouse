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



def generate_launch_description():

    is_simulation = os.getenv('IS_SIM', True)
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
            file_path=get_package_share_directory("attempt_py_moveit")
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
        package="attempt_py_moveit",
        executable="ee_point_server",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": is_simulation}
        ],
    )

    rviz_config_file = os.path.join(
        "/home/humble_ws/src/attempt_py_moveit/config",
        "rviz_config.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
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
        output="log",
        arguments=["--frame-id", "kbase_link", "--child-frame-id", "arm_base_link"],
        parameters = [
            {"use_sim_time": is_simulation}
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": is_simulation}
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
        output="log",
    )

    load_controllers = []
    for controller in [
        "kinova_arm_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="log"
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

    trajectory_server = Node(
        package="attempt_py_moveit",
        executable="trajectory_server",
        output="screen",
        parameters = [
            {"use_sim_time": is_simulation}
        ]
    )

    # kinova_interface_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('kortex_bringup'), 
    #             'launch',
    #             'gen3.launch.py'
    #         )
    #     ),
    #     launch_arguments = {
    #         'robot_ip': '192.168.1.10',
    #         'dof': '6',
    #         'launch_rviz': "false",
    #         # "gripper": "No_gripper", # Leave blank for no gripper
    #         # "gripper_joint_name": "No_gripper" # Leave blank for no gripper joint
    #     }.items()
    # )

    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        output="screen"
    )

    static_tf_2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "kbase_link", "--child-frame-id", "camera_link"],
        parameters = [
            {"use_sim_time": is_simulation}
        ]
    )

    hardware_launch_description = [
        ee_point_server, 
        move_group, 
        trajectory_server, 
        # kinova_interface_launch, 
        base_to_arm_static_tf,
        rviz_node
    ]



    return LaunchDescription(
        [
            # ee_point_server,
            robot_state_publisher,
            # # ros2_control_node,
            # rviz_node,
            # base_to_arm_static_tf,
            # static_tf_2,
            # move_group,
            # trajectory_server,
            # kinova_interface_launch,
            # realsense_node
        ]
        # + load_controllers
    )