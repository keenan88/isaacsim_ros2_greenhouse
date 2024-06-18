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

    launch_arguments = {
        "robot_ip": "xxx.yyy.zzz.www",
        "use_fake_hardware": "true",
        "gripper": "robotiq_2f_85",
        "dof": "6",
    }

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="gen3", 
            package_name="kinova_gen3_6dof_robotiq_2f_85_moveit_config",
        )
        .robot_description(mappings = launch_arguments)
        # .robot_description(
        #     file_path="config/GEN3_6DOF_VISION_URDF_ARM_V01.urdf.xacro"
        # )
        # .robot_description_semantic(
        #     file_path="config/gen3.srdf"
        # )
        # .planning_scene_monitor(
        #     publish_robot_description = True, publish_robot_description_semantic = True
        # )
        # .trajectory_execution(
        #     file_path="config/moveit_controllers.yaml"
        # )
        # .planning_pipelines(
        #     pipelines=["chomp", "pilz_industrial_motion_planner", "ompl"]
        # )
        # .moveit_cpp(
        #     file_path = "/home/humble_ws/src/antworker_moveit_bringup/config/moveit_py_params.yaml"
        # )
        .to_moveit_configs()
    )


    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": is_simulation,
                "octomap_resolution": 0.05
            }
            
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
    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="both",
    #     parameters=[
    #         moveit_config.robot_description,
    #         {"use_sim_time": is_simulation}    
    #     ],
    # )

    # ros2_controllers_path = os.path.join(
    #     get_package_share_directory("antworker_moveit_description2"),
    #     "config",
    #     "ros2_controllers.yaml",
    # )
    
    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #         ros2_controllers_path,
    #         {"use_sim_time": is_simulation}
    #     ],
    #     remappings=[
    #         ("/controller_manager/robot_description", "/robot_description"),
    #     ],
    #     output="screen",
    # )

    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "joint_state_broadcaster",
    #         "--controller-manager",
    #         "/controller_manager",
    #     ],
    # )

    # kinova_arm_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    # )

    # ctrl_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             get_package_share_directory('antworker_moveit_description2'),
    #             'launch',
    #             'spawn_controllers.launch.py'
    #         )
    #     ])
    # )

    sim_launch = [
        rviz_config_arg,
        rviz_node,
        # robot_state_publisher,
        move_group_node,
        # ros2_control_node,
        # joint_state_broadcaster_spawner,
        # kinova_arm_controller_spawner
    ]

    hw_launch = [
        rviz_config_arg,
        rviz_node,
        # robot_state_publisher,
        move_group_node,
        # ros2_control_node,    
        # joint_state_broadcaster_spawner,
        # kinova_arm_controller_spawner,
    ]

    launch = sim_launch if is_simulation else hw_launch

    return LaunchDescription(
        launch        
    )