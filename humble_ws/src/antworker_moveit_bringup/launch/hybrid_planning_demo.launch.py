import launch
import os
import sys

from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))
from k_hybrid_planning_common import (
    generate_common_hybrid_launch_description,
    get_robot_description,
    get_robot_description_semantic,
    load_yaml,
)


def generate_launch_description():
    # generate_common_hybrid_launch_description() returns a list of nodes to launch
    common_launch = generate_common_hybrid_launch_description()
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()

    # print("robot_description_semantic")
    # print(robot_description_semantic)


    # print("robot_description")
    # print(robot_description)

    # Demo node
    common_hybrid_planning_param = load_yaml(
        "moveit_hybrid_planning", "config/common_hybrid_planning_params.yaml"
    )
    # demo_node = Node(
    #     package="moveit_interface",
    #     executable="hybrid_planning_demo_node",
    #     name="hybrid_planning_demo_node",
    #     output="screen",
    #     parameters=[
    #         robot_description,
    #         robot_description_semantic,
    #         common_hybrid_planning_param,
    #         {"use_sim_time" : True}
    #     ],
    # )

    demo_node = Node(
        name="ee_point_server",
        package="antworker_moveit_bringup",
        executable="ee_point_server",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            common_hybrid_planning_param,
            load_yaml('antworker_moveit_bringup', 'config/moveit_py_params.yaml'),
            {"use_sim_time" : True}
        ],
    )

    return launch.LaunchDescription(common_launch + [demo_node])