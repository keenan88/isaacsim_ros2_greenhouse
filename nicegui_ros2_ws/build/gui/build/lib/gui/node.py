import math
import threading
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose, Twist, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import Log

from nicegui import Client, app, ui, ui_run
import datetime



class NiceGuiNode(Node):

    def __init__(self) -> None:
        super().__init__('nicegui')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        self.odom_sub = self.create_subscription(Odometry, 'odom', self.handle_pose, 1)

        self.vel_sub = self.create_subscription(Twist, 'cmd_vel', self.handle_velocity, 1)

        self.diagnostic_subscription = self.create_subscription(Log, 'rosout', self.handle_diagnostic, 10)

        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 1)

        with Client.auto_index_client:
            with ui.row().classes('items-stretch'):
                with ui.card().classes('w-44 text-center items-center'):
                    ui.label('Control').classes('text-2xl')
                    ui.joystick(color='blue', size=50,
                                on_move=lambda e: self.send_speed(float(e.y), float(e.x)),
                                on_end=lambda _: self.send_speed(0.0, 0.0))
                    #ui.label('Publish steering commands by dragging your mouse around in the blue field').classes('mt-6')

                    self.desired_pose_x_dist = 0

                    goal_pose_x_dist_inputter = ui.input(
                        label='Desired Drive Dist (m)', 
                        on_change = lambda e: setattr(self, "desired_pose_x_dist", e.value),
                    )

                    send_goal_btn = ui.button("Send goal pose").on_click(lambda e:  self.send_goal_pose(e, goal_pose_x_dist_inputter))


                with ui.card().classes('w-44 text-center items-center'):
                    ui.label('Data').classes('text-2xl')
                    ui.label('Linear Velocity').classes('text-xs mb-[-1.8em]')
                    slider_props = 'readonly selection-color=transparent'
                    self.linear = ui.slider(min=-2.5, max=2.5, step=0.05, value=0).props(slider_props)
                    # ui.label('angular velocity').classes('text-xs mb-[-1.8em]')
                    # self.angular = ui.slider(min=-1, max=1, step=0.05, value=0).props(slider_props)
                    ui.label('Position').classes('text-xs mb-[-1.4em]')
                    self.position = ui.label('---')

                    ui.label('Behaviour Status').classes('text-xs mb-[-1.4em]')
                    self.behaviour_status = ui.label('---')

                    ui.label('Controller Status').classes('text-xs mb-[-1.4em]')
                    self.controller_status = ui.label('---')

                    



                with ui.card().classes('w-96 h-96 items-center'):
                    ui.label('Visualization').classes('text-2xl')
                    with ui.scene(350, 300) as scene:
                        with scene.group() as self.robot_3d:
                            prism = [[-0.5, -0.5], [0.5, -0.5], [0.75, 0], [0.5, 0.5], [-0.5, 0.5]]
                            self.robot_object = scene.extrusion(prism, 0.4).material('#4488ff', 0.5)

    def handle_diagnostic(self, msg: Log):

        if msg.name == 'controller_server':

            date_and_time = datetime.datetime.fromtimestamp(msg.stamp.sec)

            self.controller_status.text = f'{date_and_time}:\n {msg.msg}'

        elif msg.name == 'bt_navigator':

            date_and_time = datetime.datetime.fromtimestamp(msg.stamp.sec)

            self.behaviour_status.text = f'{date_and_time}:\n {msg.msg}'

    def send_goal_pose(self, event, goal_pose_x_inputter):

        self.get_logger().info(f"ASDF: {goal_pose_x_inputter.value}")

        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = 'map'

        goal_pose.pose.position.x = float(goal_pose_x_inputter.value)

        self.goal_publisher.publish(goal_pose)



    def send_speed(self, x: float, y: float) -> None:
        msg = Twist()
        msg.linear.x = x
        # msg.angular.z = -y
        
        # self.angular.value = y
        self.cmd_vel_publisher.publish(msg)

    def handle_pose(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        quat_z = msg.pose.pose.orientation.z
        quat_w = msg.pose.pose.orientation.w

        self.position.text = f'x: {x:.2f}, y: {y:.2f}'
        self.robot_3d.move(x, y)
        self.robot_3d.rotate(0, 0, 2 * math.atan2(quat_z, quat_w))

        self.linear.value = msg.twist.twist.linear.x

    def handle_velocity(self, msg: Twist):

        pass



def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def ros_main() -> None:
    rclpy.init()
    node = NiceGuiNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖')
