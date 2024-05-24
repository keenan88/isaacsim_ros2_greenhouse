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


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="antworker", package_name="antworker_moveit_description"
        )
        .robot_description(file_path="config/antworker.urdf.xacro")
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

    # example_file = DeclareLaunchArgument(
    #     "example_file",
    #     default_value="motion_planning_python_api_tutorial.py",
    #     description="Python API tutorial file name",
    # )

    moveit_py_node = Node(
        name="motion_demo",
        package="attempt_py_moveit",
        executable="attempt_motion_planning",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("moveit2_tutorials"),
        "config",
        "motion_planning_python_api_tutorial.rviz",
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
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "kbase_link", "--child-frame-id", "arm_base_link"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("antworker_moveit_description"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="log",
    )

    load_controllers = []
    for controller in [
        # "panda_arm_controller",
        # "panda_hand_controller",
        "kinova_arm_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="log",
            )
        ]

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    return LaunchDescription(
        [
            # example_file,
            moveit_py_node,
            robot_state_publisher,
            #ros2_control_node,
            rviz_node,
            #static_tf,
            move_group_node
        ]
        #+ load_controllers
    )