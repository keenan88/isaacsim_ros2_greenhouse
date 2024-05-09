import launch
import launch_ros
from nav2_common.launch import RewrittenYaml
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = True
    
    ld = launch.LaunchDescription()

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/galactic_ws/src/antworker_navigation/rviz/nav2_config.rviz']
    )

    lifecycle_manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=['/home/galactic_ws/src/antworker_navigation/config/lifecycle_manager.yaml']
    )

    nav2_planner = launch_ros.actions.Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=['/home/galactic_ws/src/antworker_navigation/config/planner.yaml']
    )

    nav2_controller = launch_ros.actions.Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=['/home/galactic_ws/src/antworker_navigation/config/controller.yaml']
    )

    map_server = launch_ros.actions.Node(
        package = 'nav2_map_server',
        executable = 'map_server',
        name = 'map_server',
        output='screen',
        parameters = [
            {'use_sim_time': use_sim_time},
            {'yaml_filename': '/home/galactic_ws/src/antworker_navigation/maps/realsense_2m_max.yaml'}
        ]
    )

    bt_navigator = launch_ros.actions.Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=['/home/galactic_ws/src/antworker_navigation/config/bt.yaml'],
    )

    recoveries = launch_ros.actions.Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        #parameters=['/home/galactic_ws/src/antworker_navigation/config/behaviors.yaml'],
    )

    ld.add_action(lifecycle_manager)
    ld.add_action(map_server)
    ld.add_action(rviz_node)
    #ld.add_action(nav2_planner)
    #ld.add_action(nav2_controller)
    #ld.add_action(bt_navigator)
    #ld.add_action(recoveries)
    
    return ld



    
