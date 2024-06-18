import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import PlanningScene, PlanningSceneWorld, CollisionObject
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import PositionConstraint, OrientationConstraint, BoundingVolume
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from copy import deepcopy


class MoveGroupActionClientNode(Node):
    def __init__(self):
        super().__init__('move_group_action_client')

        self.static_obstacles_publisher = self.create_publisher(PlanningScene, '/monitored_planning_scene', 10)

        self.static_obstacles_publisher.publish(self.generate_side_walls())

        self.static_obstacles_publisher.publish(self.generate_robot_base())

    def generate_side_walls(self):
        wall_prim = SolidPrimitive()
        wall_prim.type = 1 # solid prim type 1 is box
        wall_prim.dimensions = [0.1, 1.0, 2.0]

        right_wall_prim_pose = Pose()
        right_wall_prim_pose.position.x = 0.6
        right_wall_prim_pose.position.y = 0.0
        right_wall_prim_pose.position.z = 0.3
        right_wall_prim_pose.orientation.x = 0.0
        right_wall_prim_pose.orientation.y = 0.0
        right_wall_prim_pose.orientation.z = 0.0 
        right_wall_prim_pose.orientation.w = 1.0

        left_wall_prim_pose = deepcopy(right_wall_prim_pose)
        left_wall_prim_pose.position.x = -left_wall_prim_pose.position.x


        side_walls_obj = CollisionObject()
        side_walls_obj.primitives.append(wall_prim)
        side_walls_obj.primitives.append(wall_prim)

        side_walls_obj.primitive_poses.append(right_wall_prim_pose)
        side_walls_obj.primitive_poses.append(left_wall_prim_pose)

        side_walls_obj.operation = b'\x02' # Operation 2 is append
        side_walls_obj.header.frame_id = "chassis_link"
        side_walls_obj.id = "side_walls"


        planning_scene_world = PlanningSceneWorld()
        planning_scene_world.collision_objects.append(side_walls_obj)

        planning_scene_diff = PlanningScene()
        planning_scene_diff.name = "test_scene_diff"
        planning_scene_diff.robot_model_name = "kinova_arm"
        planning_scene_diff.world = planning_scene_world
        planning_scene_diff.is_diff = False

        return planning_scene_diff

    def generate_robot_base(self):
        wall_prim = SolidPrimitive()
        wall_prim.type = 1 # solid prim type 1 is box
        wall_prim.dimensions = [0.1, 1.0, 2.0]

        right_wall_prim_pose = Pose()
        right_wall_prim_pose.position.x = 0.6
        right_wall_prim_pose.position.y = 0.0
        right_wall_prim_pose.position.z = 0.3
        right_wall_prim_pose.orientation.x = 0.0
        right_wall_prim_pose.orientation.y = 0.0
        right_wall_prim_pose.orientation.z = 0.0 
        right_wall_prim_pose.orientation.w = 1.0

        left_wall_prim_pose = deepcopy(right_wall_prim_pose)
        left_wall_prim_pose.position.x = -left_wall_prim_pose.position.x


        side_walls_obj = CollisionObject()
        side_walls_obj.primitives.append(wall_prim)
        side_walls_obj.primitives.append(wall_prim)

        side_walls_obj.primitive_poses.append(right_wall_prim_pose)
        side_walls_obj.primitive_poses.append(left_wall_prim_pose)

        side_walls_obj.operation = b'\x02' # Operation 2 is append
        side_walls_obj.header.frame_id = "chassis_link"
        side_walls_obj.id = "side_walls"


        planning_scene_world = PlanningSceneWorld()
        planning_scene_world.collision_objects.append(side_walls_obj)

        planning_scene_diff = PlanningScene()
        planning_scene_diff.name = "test_scene_diff"
        planning_scene_diff.robot_model_name = "kinova_arm"
        planning_scene_diff.world = planning_scene_world
        planning_scene_diff.is_diff = False

        return planning_scene_diff
    



def main(args=None):
    rclpy.init(args=args)
    move_group_node = MoveGroupActionClientNode()

    rclpy.spin(move_group_node)

    

if __name__ == '__main__':
    main()