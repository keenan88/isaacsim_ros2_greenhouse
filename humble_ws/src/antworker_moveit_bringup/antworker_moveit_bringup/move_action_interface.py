import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import PositionConstraint, OrientationConstraint, BoundingVolume
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

class MoveGroupActionClientNode:
    def __init__(self):
        self.node = rclpy.create_node('move_action_action_client')

        self.move_group_action_client = ActionClient(self.node, MoveGroup, 'move_action')

        while not self.move_group_action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().info('Action server not available, waiting again...')

    def send_goal(self):
        goal_msg = MoveGroup.Goal()

        goal_msg.planning_options.plan_only = True
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = False

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
        bounding_prim.dimensions = [0.1, 0.1, 0.1]

        # is the bounding box in the arm_base_link frame or the end effector frame?
        bounding_prim_pose = Pose()
        bounding_prim_pose.position.x = 0.2
        bounding_prim_pose.position.y = 0.01
        bounding_prim_pose.position.z = 0.9
        bounding_prim_pose.orientation.x = 0.0
        bounding_prim_pose.orientation.y = 0.0
        bounding_prim_pose.orientation.z = 0.0
        bounding_prim_pose.orientation.w = 1.0

        bounding_volume.primitives.append(bounding_prim)
        bounding_volume.primitive_poses.append(bounding_prim_pose)

        ee_pose_constraint.constraint_region = bounding_volume
        

        ee_orient_constraint = OrientationConstraint()
        ee_orient_constraint.header.frame_id = "arm_base_link"
        ee_orient_constraint.link_name = "end_effector_link"
        ee_orient_constraint.orientation.x = 0.0
        ee_orient_constraint.orientation.x = 0.0
        ee_orient_constraint.orientation.x = 1.0
        ee_orient_constraint.orientation.x = 0.0

        constraints = Constraints()
        constraints.position_constraints.append(ee_pose_constraint)
        constraints.orientation_constraints.append(ee_orient_constraint)

        goal_msg.request.goal_constraints.append(constraints)


        

        

        self.future = self.move_group_action_client.send_goal_async(goal_msg, feedback_callback = self.feedback_callback)

        rclpy.spin_until_future_complete(self.node, self.future)

        if self.future.result() is not None:
            self.node.get_logger().info('Goal succeeded!')
        else:
            self.node.get_logger().info('Goal failed!')

    def feedback_callback(self, feedback_msg):
        self.node.get_logger().info('Received feedback: {0}'.format(feedback_msg))

    def destroy_node(self):
        self.move_group_action_client.destroy()
        self.node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    move_group_node = MoveGroupActionClientNode()

    # Send a goal to the action server
    move_group_node.send_goal()

    # Cleanup
    move_group_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
