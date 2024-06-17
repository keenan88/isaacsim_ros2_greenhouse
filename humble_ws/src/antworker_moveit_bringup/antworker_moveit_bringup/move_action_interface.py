import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import PositionConstraint, OrientationConstraint, BoundingVolume
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive


class MoveGroupActionClientNode(Node):
    def __init__(self):
        super().__init__('move_group_action_client')

        self.move_group_action_client = ActionClient(self, MoveGroup, 'move_action')

        while not self.move_group_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Action server not available, waiting again...')

        goal_msg = self.generate_goal_msg()

        self.move_group_action_client.wait_for_server()

        self.future = self.move_group_action_client.send_goal_async(goal_msg, feedback_callback = self.feedback_callback)

        self.future.add_done_callback(self.goal_response_callback)

        # self.move_group_action_client.wait_for_server()

        # goal_msg = self.generate_goal_msg2()

        # self.future = self.move_group_action_client.send_goal_async(goal_msg, feedback_callback = self.feedback_callback)

        # self.future.add_done_callback(self.goal_response_callback)

        

    def generate_goal_msg(self):
        goal_msg = MoveGroup.Goal()

        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 10
        goal_msg.planning_options.replan_delay = 1.0

        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        goal_msg.request.num_planning_attempts = 100
        goal_msg.request.allowed_planning_time = 1.0 # seconds
        goal_msg.request.group_name = "kinova_arm"
        goal_msg.request.planner_id = "PTP"

        ee_pose_constraint = PositionConstraint()
        ee_pose_constraint.header.frame_id = "arm_base_link"
        ee_pose_constraint.link_name = "end_effector_link"
        ee_pose_constraint.target_point_offset.x = 0.0
        ee_pose_constraint.target_point_offset.y = 0.0
        ee_pose_constraint.target_point_offset.z = 0.0

        bounding_volume = BoundingVolume()

        bounding_prim = SolidPrimitive()
        bounding_prim.type = 1 # 1 is box
        bounding_prim.dimensions = [0.01, 0.01, 0.01]

        # is the bounding box in the arm_base_link frame or the end effector frame?
        bounding_prim_pose = Pose()
        bounding_prim_pose.position.x = 0.0
        bounding_prim_pose.position.y = 0.7
        bounding_prim_pose.position.z = 0.2
        bounding_prim_pose.orientation.x = 0.0
        bounding_prim_pose.orientation.y = 0.0
        bounding_prim_pose.orientation.z = 0.0
        bounding_prim_pose.orientation.w = 1.0

        bounding_volume.primitives.append(bounding_prim)
        bounding_volume.primitive_poses.append(bounding_prim_pose)

        ee_pose_constraint.constraint_region = bounding_volume
        

        ee_orient_constraint = OrientationConstraint()
        ee_orient_constraint.header.frame_id = "chassis_link"
        ee_orient_constraint.link_name = "end_effector_link"
        ee_orient_constraint.orientation.x = 0.0
        ee_orient_constraint.orientation.y = 1.0
        ee_orient_constraint.orientation.z = 0.0
        ee_orient_constraint.orientation.w = 0.0

        constraints = Constraints()
        constraints.position_constraints.append(ee_pose_constraint)
        constraints.orientation_constraints.append(ee_orient_constraint)

        goal_msg.request.goal_constraints.append(constraints)

        return goal_msg

    def generate_goal_msg2(self):
        goal_msg = MoveGroup.Goal()

        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 10
        goal_msg.planning_options.replan_delay = 1.0

        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        goal_msg.request.num_planning_attempts = 100
        goal_msg.request.allowed_planning_time = 1.0 # seconds
        goal_msg.request.group_name = "kinova_arm"
        goal_msg.request.planner_id = "PTP"

        ee_pose_constraint = PositionConstraint()
        ee_pose_constraint.header.frame_id = "chassis_link"
        ee_pose_constraint.link_name = "end_effector_link"
        ee_pose_constraint.target_point_offset.x = 0.0
        ee_pose_constraint.target_point_offset.y = 0.0
        ee_pose_constraint.target_point_offset.z = 0.0

        bounding_volume = BoundingVolume()

        bounding_prim = SolidPrimitive()
        bounding_prim.type = 1 # 1 is box
        bounding_prim.dimensions = [0.01, 0.01, 0.01]

        # is the bounding box in the arm_base_link frame or the end effector frame?
        bounding_prim_pose = Pose()
        bounding_prim_pose.position.x = -0.5
        bounding_prim_pose.position.y = 0.3
        bounding_prim_pose.position.z = 0.1
        bounding_prim_pose.orientation.x = 0.0
        bounding_prim_pose.orientation.y = 0.0
        bounding_prim_pose.orientation.z = 0.0
        bounding_prim_pose.orientation.w = 1.0

        bounding_volume.primitives.append(bounding_prim)
        bounding_volume.primitive_poses.append(bounding_prim_pose)

        ee_pose_constraint.constraint_region = bounding_volume
        

        ee_orient_constraint = OrientationConstraint()
        ee_orient_constraint.header.frame_id = "chassis_link"
        ee_orient_constraint.link_name = "end_effector_link"
        ee_orient_constraint.orientation.x = 0.0
        ee_orient_constraint.orientation.y = 1.0
        ee_orient_constraint.orientation.z = 0.0
        ee_orient_constraint.orientation.w = 0.0

        constraints = Constraints()
        constraints.position_constraints.append(ee_pose_constraint)
        constraints.orientation_constraints.append(ee_orient_constraint)

        goal_msg.request.goal_constraints.append(constraints)

        return goal_msg

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))

    def get_result_callback(self, future):
        result = future.result().result
        #self.get_logger().info('Result: {0}'.format(result))

        print("planned trajectory:")

        for traj_point in result.planned_trajectory.joint_trajectory.points:
            print(traj_point)
            print()

        print()
        print()
        print()

        print("executed trajectory:")

        for traj_point in result.executed_trajectory.joint_trajectory.points:
            print(traj_point)
            print()

        

        rclpy.shutdown()



def main(args=None):
    rclpy.init(args=args)
    move_group_node = MoveGroupActionClientNode()

    rclpy.spin(move_group_node)

    

if __name__ == '__main__':
    main()