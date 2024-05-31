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
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


import rclpy
from rclpy.node import Node

class PointServer(Node):
    def __init__(self):
        super().__init__('PointServer')

        self.logger = get_logger("moveit_py.pose_goal")

        # instantiate MoveItPy instance and get planning component
        self.kinova = MoveItPy(node_name = "moveit_py")
        self.kinova_arm = self.kinova.get_planning_component("kinova_arm")
        self.logger.info("MoveItPy instance created")

        self.planning_scene_monitor = self.kinova.get_planning_scene_monitor()

        with self.planning_scene_monitor.read_write() as scene:

            dimensions = [0.1, 0.1, 0.1]
            collision_object = CollisionObject()
            collision_object.header.frame_id = "arm_base_link"
            collision_object.id = "boxes"

            box_pose = Pose()
            box_pose.position.x = 1.0
            box_pose.position.y = 0.1
            box_pose.position.z = 0.6

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = dimensions

            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(box_pose)
            collision_object.operation = CollisionObject.ADD


            scene.apply_collision_object(collision_object)
            scene.current_state.update()  # Important to ensure the scene is updated

        ###########################################################################
        # Plan 1 - set states with predefined string
        ###########################################################################

        self.logger.info("\nMOVING ARM TO PRESET POINT\n")

        # set plan start state using predefined state
        self.kinova_arm.set_start_state_to_current_state()

        # set pose goal using predefined state
        self.kinova_arm.set_goal_state(configuration_name="jagged")

        # plan to goal
        self.plan_and_execute(self.kinova, self.kinova_arm, sleep_time=3.0)    

        # ###########################################################################
        # # Plan 3 - set goal state with PoseStamped message
        # ###########################################################################

        self.logger.info("\nMOVING ARM TO CUSTOM POINT\n")

        # set plan start state to current state
        self.kinova_arm.set_start_state_to_current_state()

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
        self.kinova_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="end_effector_link")

        # plan to goal
        self.plan_and_execute(self.kinova, self.kinova_arm, sleep_time=3.0)

        
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
    node = PointServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()