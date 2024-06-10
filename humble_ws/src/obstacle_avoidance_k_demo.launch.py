import os, yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():

    # Command-line arguments
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="/home/humble_ws/src/antworker_moveit_bringup/config/rviz_config.rviz",
        description="RViz configuration file",
    )

    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database flag"
    )

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="antworker", 
            package_name="antworker_moveit_description",
        )
        .robot_description(
            file_path="config/antworker.urdf.xacro"
        )
        .robot_description_semantic(file_path="config/antworker.srdf")
        .planning_scene_monitor(
            publish_robot_description = True, publish_robot_description_semantic = True
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["chomp", "pilz_industrial_motion_planner"] # stomp, ompl
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
            {"use_sim_time": True}
            
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
            {"use_sim_time": True}
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
            {"use_sim_time": True}    
        ],
    )

    # ros2_control using FakeSystem as hardware
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
            {"use_sim_time": True}
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

    global_planner_param = load_yaml(
        "antworker_moveit_bringup", "config/global_planner.yaml"
    )
    local_planner_param = load_yaml(
        "antworker_moveit_bringup", "config/local_planner.yaml"
    )
    hybrid_planning_manager_param = load_yaml(
        "antworker_moveit_bringup", "config/hybrid_planning_manager.yaml"
    )

    common_hybrid_planning_param = load_yaml(
        "antworker_moveit_bringup", "config/common_hybrid_planning_params.yaml"
    )

    # Generate launch description with multiple components
    container = ComposableNodeContainer(
        name="hybrid_planning_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::GlobalPlannerComponent",
                name="global_planner",
                parameters=[
                    global_planner_param,
                    moveit_config.to_dict(),
                    common_hybrid_planning_param
                ],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::LocalPlannerComponent",
                name="local_planner",
                parameters=[
                    local_planner_param,
                    moveit_config.to_dict(),
                    common_hybrid_planning_param
                ],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::HybridPlanningManager",
                name="hybrid_planning_manager",
                parameters=[
                    hybrid_planning_manager_param,
                    common_hybrid_planning_param
                ],
            ),
        ],
        output="screen",
    )

    

    return LaunchDescription(
        [
            rviz_config_arg,
            rviz_node,
            robot_state_publisher,
            move_group_node,

            ros2_control_node,
            joint_state_broadcaster_spawner,
            panda_arm_controller_spawner,
            
            container
        ]
    )