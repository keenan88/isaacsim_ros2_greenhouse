import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    is_simulation = os.getenv('USE_SIM')
    if is_simulation == 'False': is_simulation = False
    else: is_simulation = True

    print("is_simulation: ", is_simulation)

    # Command-line arguments
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="/home/humble_ws/src/antworker_moveit_bringup/config/rviz_config.rviz",
        description="RViz configuration file",
    )

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="antworker", 
            package_name="antworker_moveit_description",
        )
        .robot_description(
            file_path="config/antworker.urdf.xacro"
        )
        .robot_description_semantic(
            file_path="config/antworker.srdf"
        )
        .planning_scene_monitor(
            publish_robot_description = True, publish_robot_description_semantic = True
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml"
        )
        .planning_pipelines(
            pipelines=["pilz_industrial_motion_planner"] # stomp, ompl
        )
        .moveit_cpp(
            file_path = "/home/humble_ws/src/antworker_moveit_bringup/config/moveit_py_params.yaml"
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": is_simulation}
            
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_base],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": is_simulation}
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": is_simulation}    
        ],
    )

    # ros2_control using FakeSystem as hardware
    # ros2_controllers_path = os.path.join(
    #     get_package_share_directory("antworker_moveit_description"),
    #     "config",
    #     "ros2_controllers.yaml",
    # )

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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["kinova_arm_controller", "-c", "/controller_manager"],
    )

    joint_command_forwarder = Node(
        package = "antworker_moveit_bringup",
        executable = "joint_command_forwarder",
        output = "screen"
    )

    dummy_wheel_joint_publisher = Node(
        package = "antworker_moveit_bringup",
        executable = "dummy_wheel_joint_publisher",
        output = "screen"
    )

    ctrl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('antworker_moveit_description'),
                'launch',
                'spawn_controllers.launch.py'
            )
        ])
    )

    sim_launch = [
        rviz_config_arg,
        rviz_node,
        robot_state_publisher,
        move_group_node,
        ros2_control_node,
        #joint_state_broadcaster_spawner,
        #panda_arm_controller_spawner,
        joint_command_forwarder,
        ctrl_launch
    ]

    hw_launch = [
        rviz_config_arg,
        rviz_node,
        robot_state_publisher,
        move_group_node,
        # ros2_control_node,    
        # joint_state_broadcaster_spawner,
        # panda_arm_controller_spawner,
        dummy_wheel_joint_publisher
    ]

    launch = sim_launch if is_simulation else hw_launch

    return LaunchDescription(
        launch        
    )