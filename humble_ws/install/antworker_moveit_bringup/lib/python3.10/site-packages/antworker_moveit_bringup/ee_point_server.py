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
from geometry_msgs.msg import PoseStamped
import numpy as np


import rclpy
from rclpy.node import Node

class PointServer(Node):
    def __init__(self):
        super().__init__('PointServer')

        self.logger = get_logger("moveit_py.pose_goal")

        self.kinova = MoveItPy(node_name = "moveit_py")
        self.kinova_planner = self.kinova.get_planning_component("kinova_arm")
        self.planning_scene_monitor = self.kinova.get_planning_scene_monitor()

        with self.planning_scene_monitor.read_write() as scene:

            dimensions = [0.1, 0.1, 0.1]
            collision_object = CollisionObject()
            collision_object.header.frame_id = "arm_base_link"
            collision_object.id = "boxes"

            box_pose = Pose()
            box_pose.position.x = -0.125
            box_pose.position.y = 0.0
            box_pose.position.z = 0.6

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = dimensions

            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(box_pose)
            collision_object.operation = CollisionObject.ADD

            box_pose = Pose()
            box_pose.position.x = 0.125
            box_pose.position.y = 0.0
            box_pose.position.z = 0.6

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = dimensions

            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(box_pose)
            collision_object.operation = CollisionObject.ADD


            scene.apply_collision_object(collision_object)
            scene.current_state.update() 

        # self.logger.info("\n\n\n")

        # self.kinova_planner.set_start_state_to_current_state()
        # self.kinova_planner.set_goal_state(configuration_name='extended')

        # plan_result = self.kinova_planner.plan()

        # if plan_result:

        #     self.logger.info("Number of trajectory points: ")
        #     self.logger.info(f'{len(plan_result.trajectory.get_robot_trajectory_msg().joint_trajectory.points)}')
        #     for traj_point in plan_result.trajectory.get_robot_trajectory_msg().joint_trajectory.points:            
        #        self.logger.info(f'{np.round(traj_point.positions, 4)}')

        #     self.kinova.execute(plan_result.trajectory, controllers=[])

        # time.sleep(5) # TODO - find a better way of validating that movement is finished than just blind time

        # self.logger.info("\n\n\n")
        self.kinova_planner.set_start_state_to_current_state()

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "arm_base_link"        
        pose_goal.pose.orientation.x = 0.0
        pose_goal.pose.orientation.y = 0.0
        pose_goal.pose.orientation.z = 1.0
        pose_goal.pose.orientation.w = 0.0
        pose_goal.pose.position.x = 0.2
        pose_goal.pose.position.y = 0.01
        pose_goal.pose.position.z = 0.9
        self.kinova_planner.set_goal_state(pose_stamped_msg = pose_goal, pose_link = "end_effector_link")

        plan_result = self.kinova_planner.plan()

        if plan_result:
            self.kinova.execute(plan_result.trajectory, controllers=[])


        time.sleep(5) # TODO - find a better way of validating that movement is finished than just blind time

        self.logger.info("\n\n\n")
        self.kinova_planner.set_start_state_to_current_state()

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "arm_base_link"        
        pose_goal.pose.orientation.x = 0.0
        pose_goal.pose.orientation.y = 0.0
        pose_goal.pose.orientation.z = 1.0
        pose_goal.pose.orientation.w = 0.0
        pose_goal.pose.position.x = -0.4
        pose_goal.pose.position.y = 0.01
        pose_goal.pose.position.z = 0.9
        self.kinova_planner.set_goal_state(pose_stamped_msg = pose_goal, pose_link = "end_effector_link")

        plan_result = self.kinova_planner.plan()

        if plan_result:
            self.kinova.execute(plan_result.trajectory, controllers=[])

    

def main(args=None):
    rclpy.init(args=args)
    node = PointServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()