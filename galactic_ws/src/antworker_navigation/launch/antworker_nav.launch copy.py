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
        arguments=['-d', '/home/humble_ws/src/antworker_navigation/rviz/nav2_config.rviz']
    )

    #lifecycle_nodes = ['map_server', 'amcl', 'localizer', 'planner_server', 'controller_server', 'recoveries_server', 'bt_navigator']
    #lifecycle_nodes = ['controller_server', 'planner_server', 'map_server']
    lifecycle_nodes = []
    #lifecycle_nodes = ['planner_server']

    nav2_controller_node = launch_ros.actions.Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    nav2_planner_node = launch_ros.actions.Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    map_server = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        arguments=['-f', '/home/humble_ws/src/antworker_navigation/maps/realsense_2m_max.yaml']
    )

    simtime_la = DeclareLaunchArgument(
        'use_sim_time', default_value = True,
        description='Use simulation (IsaacSim) clock if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    

    asdf = RewrittenYaml(
        source_file='/home/humble_ws/src/antworker_navigation/config/lifecycle_manager.yaml',
        param_rewrites={
            'use_sim_time': use_sim_time,
            # 'node_names': ['map_server']
        },
        convert_types = True
    )
    
    print(asdf)

    lifecycle_manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['']
            }
        ]
    )

    #ld.add_action(map_server)

    ld.add_action(lifecycle_manager)   
    
    #ld.add_action(nav2_planner_node)

    #ld.add_action(nav2_controller_node)

    #ld.add_action(rviz_node)

    return ld



    
