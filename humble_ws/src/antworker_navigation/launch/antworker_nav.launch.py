import launch
import launch_ros

def generate_launch_description():
    return launch.LaunchDescription()

if __name__ == '__main__':
    ld = generate_launch_description()

    # Add the rviz node
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        #arguments=['-d', '/path/to/your/rviz/config/file.rviz']
    )

    ld.add_action(rviz_node)
