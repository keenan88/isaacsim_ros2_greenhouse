#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

import time

# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)

import rclpy
from rclpy.node import Node

class EmptyNode(Node):
    def __init__(self):
        super().__init__('empty_node')
        self.get_logger().info('Empty node has been started.')

        self.logger = get_logger("moveit_py.pose_goal")

        # instantiate MoveItPy instance and get planning component
        self.panda = MoveItPy(node_name = "moveit_py")
        self.panda_arm = self.panda.get_planning_component("kinova_arm")
        self.logger.info("MoveItPy instance created")

        ###########################################################################
        # Plan 1 - set states with predefined string
        ###########################################################################

        self.logger.info("\nMOVING ARM TO PRESET POINT\n")

        # set plan start state using predefined state
        self.panda_arm.set_start_state(configuration_name="extended")

        # set pose goal using predefined state
        self.panda_arm.set_goal_state(configuration_name="pose3")

        # plan to goal
        self.plan_and_execute(self.panda, self.panda_arm, sleep_time=3.0)    

        # ###########################################################################
        # # Plan 3 - set goal state with PoseStamped message
        # ###########################################################################

        self.logger.info("\nMOVING ARM TO CUSTOM POINT\n")

        # set plan start state to current state
        self.panda_arm.set_start_state_to_current_state()

        # set pose goal with PoseStamped message
        from geometry_msgs.msg import PoseStamped

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "arm_base_link"        
        pose_goal.pose.orientation.x = -0.322
        pose_goal.pose.orientation.y = 0.250
        pose_goal.pose.orientation.z = 0.627
        pose_goal.pose.orientation.w = 0.664
        pose_goal.pose.position.x = -0.290
        pose_goal.pose.position.y = 0.334
        pose_goal.pose.position.z = 0.9
        self.panda_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="end_effector_link")

        # plan to goal
        self.plan_and_execute(self.panda, self.panda_arm, sleep_time=3.0)

        
        self.logger.info("DONE MOVING ARM AROUND")

    def plan_and_execute(self, robot, planning_component, single_plan_parameters=None, multi_plan_parameters=None, sleep_time=0.0):
        self.logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
            plan_result = planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            plan_result = planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            plan_result = planning_component.plan()

        # execute the plan
        if plan_result:
            self.logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            robot.execute(robot_trajectory, controllers=[])
        else:
            self.logger.error("Planning failed")

        time.sleep(sleep_time)


def main(args=None):
    rclpy.init(args=args)
    node = EmptyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()