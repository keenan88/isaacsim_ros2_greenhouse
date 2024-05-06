#!/usr/bin/env python3
import time
import threading
import numpy as np

import rospy

from actionlib import SimpleActionServer
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, BatteryState
from gazebo_msgs.msg import ContactsState, LinkStates
from iris_msgs.msg import MoveShiftWheelsFeedback, MoveShiftWheelsResult, MoveShiftWheelsAction, IrisStatus, IrisMotors
from std_msgs.msg import Bool, Float64
from tf.transformations import euler_from_quaternion


class MovementDynamics:
    def __init__(self, k1, k2, dt):
        self._A = np.array([[k1*dt, 0.0], [dt, 0.0]])
        self._B = np.array([[k2*dt], [0.0]])
        self._C = np.array([[1.0, 0.0]])
        self._x = np.zeros([2, 1])

    def __call__(self, u):
        self._x += self._A @ self._x + self._B * u

    @property
    def state(self):
        return (self._C @ self._x)[0, 0]

    def reset(self):
        self._x = np.zeros(2)


class BaseControllerNode(object):

    def __init__(self, name):
        rospy.init_node(name, anonymous=True)

        # Constants
        self._ground_distance = 0.05
        self._wheel_change_time = 3.0

        # Internal state memory
        # Contacts in simulation can be messy.
        # It's better to filter them with simple moving average
        self._front_bumper_memory = [0]*10
        self._rear_bumper_memory = [0]*10

        # Initialise IrisStatus message
        self._status_lock = threading.Lock()
        self._iris_status = IrisStatus()
        self._iris_status.throttle_lever_position = 0
        self._iris_status.is_throttle_lever_reversed = False
        self._iris_status.is_estop_button_pressed = False
        self._iris_status.is_up_button_pressed = False
        self._iris_status.is_down_button_pressed = False

        self._iris_status.is_caster_wheel_lowered = False
        self._iris_status.is_left_right_wheel_lowered = False
        self._iris_status.is_left_right_wheel_raised = True
        self._iris_status.is_mast_lowered = True
        self._iris_status.is_front_bumper_pressed = False
        self._iris_status.is_rear_bumper_pressed = False
        self._iris_status.is_ultrasound_detecting_left = False
        self._iris_status.is_ultrasound_detecting_right = False
        self._iris_status.is_tilted = False
        self._iris_status.is_12V_on = False
        self._iris_status.is_case_fan_on = False
        self._iris_status.is_alarm_speaker_on = False
        self._iris_status.is_status_light_on = False
        self._iris_status.is_caster_wheel_lowering = False
        self._iris_status.is_caster_wheel_raising = False
        self._iris_status.is_left_right_wheel_lowering = False
        self._iris_status.is_left_right_wheel_raising = False
        self._iris_status.is_bumpers_enabled = True
        self._iris_status.is_under_manual_control = False

        self._fr_vel_std_dev = 0.09788538145557155
        self._fr_vel_mean_noise = 0.14146391795242577
        self._fr_pos_std_dev = 0.32992123842359294
        self._fr_pos_mean_noise = -0.0006690635233445121

        self._lr_vel_std_dev = 0.09788538145557155
        self._lr_vel_mean_noise = 0.14146391795242577
        self._lr_pos_std_dev = 0.32992123842359294
        self._lr_pos_mean_noise = -0.0006690635233445121

        self._fr_throttle = Float64()
        self._lr_throttle = Float64()

        self._dt = 1./10.

        self._fr_dynamics = MovementDynamics(
            k1=-0.4824489105724535,
            k2=0.008531802452099179,
            dt=self._dt
        )

        self._lr_dynamics = MovementDynamics(
            k1=-0.8961901521437101,
            k2=0.0016650502678675484,
            dt=self._dt
        )

        # Initialise rail messages
        self._yaw = 0.0
        self._link_state_sub_cnt = 0
        self._cmd_vel = Twist()
        self._rail_lock = threading.Lock()
        self._front_above_rail = Bool()
        self._rear_above_rail = Bool()

        # Subscribers
        self.front_rfid_laser_sub = rospy.Subscriber('gazebo/front_rfid_laser',
                                                     Range, self._front_rfid_laser_callback)
        self.rear_rfid_laser_sub = rospy.Subscriber('gazebo/rear_rfid_laser',
                                                    Range, self._rear_rfid_laser_callback)
        self._iris_front_bumper_subscriber = rospy.Subscriber(
            'gazebo/front_bumper', ContactsState, self._front_bumper_callback)
        self._iris_rear_bumper_subscriber = rospy.Subscriber(
            'gazebo/rear_bumper', ContactsState, self._rear_bumper_callback)
        self._fr_throttle_sub = rospy.Subscriber(
            'iris/forward_reverse_motor_throttle/filtered', Float64, self._fr_throttle_callback)
        self._lr_throttle_sub = rospy.Subscriber(
            'iris/left_right_motor_throttle/filtered', Float64, self._lr_throttle_callback)
        self._link_state_subscriber = rospy.Subscriber(
            "gazebo/link_states", LinkStates, self._link_state_callback)

        # Publishers
        self._iris_status_publisher = rospy.Publisher(
            'iris/status', IrisStatus, queue_size=1)
        self._iris_motors_publisher = rospy.Publisher(
            'iris/motors', IrisMotors, queue_size=1)
        self._iris_battery_publisher = rospy.Publisher(
            'iris/battery', BatteryState, queue_size=1)
        self._cmd_vel_publisher = rospy.Publisher(
            'gazebo/iris/cmd_vel', Twist, queue_size=5)
        self._front_above_rail_publisher = rospy.Publisher(
            'iris/front_above_rail', Bool, queue_size=5)
        self._rear_above_rail_publisher = rospy.Publisher(
            'iris/rear_above_rail', Bool, queue_size=5)

        # Timers
        self._iris_status_timer = rospy.Timer(rospy.Duration(
            1./4.), self._iris_status_callback)

        self._iris_rail_timer = rospy.Timer(rospy.Duration(
            1./10.), self._iris_rails_callback)

        # Timers
        self._dynamics_timer = rospy.Timer(
            rospy.Duration(self._dt), self._dynamics_callback)

        # Action server
        self._action_server = SimpleActionServer(
            'move_shift_wheels', MoveShiftWheelsAction,
            execute_cb=self._move_shift_wheels_action_callback, auto_start=False)

        self._feedback = MoveShiftWheelsFeedback()
        self._result = MoveShiftWheelsResult()
        self._action_server.register_preempt_callback(self._preempt_callback)
        self._action_server.start()

        rospy.loginfo(f'{name} node started')

    def _fr_throttle_callback(self, data):
        self._fr_throttle = data

    def _lr_throttle_callback(self, data):
        self._lr_throttle = data

    def _link_state_callback(self, data):
        if self._link_state_sub_cnt >= 10:
            self._link_state_sub_cnt = 0
            base_link_idx = data.name.index('iris::base_link')

            fr_vel_noise = np.random.normal(
                0.0, self._fr_vel_std_dev) * self._fr_vel_mean_noise
            fr_pos_noise = np.random.normal(
                0.0, self._fr_pos_std_dev) * self._fr_pos_mean_noise
            lr_vel_noise = np.random.normal(
                0.0, self._lr_vel_std_dev) * self._lr_vel_mean_noise
            lr_pos_noise = np.random.normal(
                0.0, self._lr_pos_std_dev) * self._lr_pos_mean_noise

            motors_msg = IrisMotors()
            motors_msg.forward_reverse_position_on_pipe = data.pose[
                base_link_idx].position.x + fr_pos_noise
            motors_msg.forward_reverse_position_on_concrete = data.pose[
                base_link_idx].position.x + fr_pos_noise
            motors_msg.left_right_position = data.pose[base_link_idx].position.y + lr_pos_noise

            if self._iris_status.is_left_right_wheel_lowered:
                motors_msg.forward_reverse_velocity_on_pipe = 0.0
                motors_msg.forward_reverse_velocity_on_concrete = 0.0
                motors_msg.left_right_velocity = data.twist[base_link_idx].linear.y + lr_vel_noise

            else:
                motors_msg.forward_reverse_velocity_on_pipe = data.twist[
                    base_link_idx].linear.x + fr_vel_noise
                motors_msg.forward_reverse_velocity_on_concrete = data.twist[
                    base_link_idx].linear.x + fr_vel_noise
                motors_msg.left_right_velocity = 0.0

            _, _, self._yaw = euler_from_quaternion([
                data.pose[base_link_idx].orientation.x,
                data.pose[base_link_idx].orientation.y,
                data.pose[base_link_idx].orientation.z,
                data.pose[base_link_idx].orientation.w
            ])

            self._iris_motors_publisher.publish(motors_msg)
        self._link_state_sub_cnt += 1

    def _dynamics_callback(self, *args):
        self._fr_dynamics(self._fr_throttle.data)
        self._lr_dynamics(self._lr_throttle.data)

        cmd_vel_filtered = Twist()
        with self._status_lock:
            if self._iris_status.is_left_right_wheel_lowered:
                cmd_vel_filtered.linear.y = self._lr_dynamics.state

            elif self._iris_status.is_left_right_wheel_raised:
                cmd_vel_filtered.linear.x = self._fr_dynamics.state

        cmd_vel_filtered.angular.z = -self._yaw
        self._cmd_vel_publisher.publish(cmd_vel_filtered)

    def _iris_status_callback(self, event):
        with self._status_lock:
            self._iris_status_publisher.publish(self._iris_status)
            self._iris_battery_publisher.publish(BatteryState())

    def _iris_rails_callback(self, event):
        with self._rail_lock:
            self._front_above_rail_publisher.publish(self._front_above_rail)
            self._rear_above_rail_publisher.publish(self._rear_above_rail)

    def _front_rfid_laser_callback(self, msg):
        distance = msg.range
        with self._rail_lock:
            if distance < self._ground_distance and not self._front_above_rail.data:
                self._front_above_rail.data = True
            elif distance > self._ground_distance and self._front_above_rail.data:
                self._front_above_rail.data = False

    def _rear_rfid_laser_callback(self, msg):
        distance = msg.range
        with self._rail_lock:
            if distance < self._ground_distance and not self._rear_above_rail.data:
                self._rear_above_rail.data = True
            elif distance > self._ground_distance and self._rear_above_rail.data:
                self._rear_above_rail.data = False

    def _front_bumper_callback(self, msg):
        # Check if there is contact with anything
        observation = len(msg.states) > 0

        self._front_bumper_memory.pop(0)
        self._front_bumper_memory.append(observation)
        average = sum(self._front_bumper_memory) / \
            len(self._front_bumper_memory)

        with self._status_lock:
            if average > 0.2:
                self._iris_status.is_front_bumper_pressed = True
            else:
                self._iris_status.is_front_bumper_pressed = False

    def _rear_bumper_callback(self, msg):
        observation = len(msg.states) > 0

        self._rear_bumper_memory.pop(0)
        self._rear_bumper_memory.append(observation)
        average = sum(self._rear_bumper_memory) / len(self._rear_bumper_memory)

        with self._status_lock:
            if average > 0.2:
                self._iris_status.is_rear_bumper_pressed = True
            else:
                self._iris_status.is_rear_bumper_pressed = False

    def _move_shift_wheels_action_callback(self, goal):
        rospy.loginfo(f'[MoveShiftWheelsAction] moving to {goal.position}...')
        success = True

        if self._front_above_rail.data or self._rear_above_rail.data:
            rospy.loginfo('[MoveShiftWheelsAction] in row, cannot move wheels')
            success = False

        if success:
            if goal.position == 'raised':
                rospy.loginfo('[MoveShiftWheelsAction] raising...')
                caster_wheel_lowered = False
                with self._status_lock:
                    caster_wheel_lowered = self._iris_status.is_left_right_wheel_lowered
                    self._iris_status.is_left_right_wheel_lowered = False
                    self._iris_status.is_left_right_wheel_raised = False
                    self._iris_status.is_left_right_wheel_lowering = False
                    self._iris_status.is_left_right_wheel_raising = True

                if caster_wheel_lowered:
                    time.sleep(self._wheel_change_time)

                with self._status_lock:
                    self._iris_status.is_left_right_wheel_raised = True
                    self._iris_status.is_left_right_wheel_raising = False

            elif goal.position == 'lowered':
                rospy.loginfo('[MoveShiftWheelsAction] lowering...')
                caster_wheel_lowered = False
                with self._status_lock:
                    caster_wheel_lowered = self._iris_status.is_left_right_wheel_raised
                    self._iris_status.is_left_right_wheel_raised = False
                    self._iris_status.is_left_right_wheel_lowered = False
                    self._iris_status.is_left_right_wheel_lowering = True
                    self._iris_status.is_left_right_wheel_raising = False

                if caster_wheel_lowered:
                    time.sleep(self._wheel_change_time)

                with self._status_lock:
                    self._iris_status.is_left_right_wheel_lowered = True
                    self._iris_status.is_left_right_wheel_lowering = False

            else:
                rospy.logerr(
                    f'[MoveShiftWheelsAction] invalid position {goal.position}...')
                rospy.loginfo(
                    '[MoveShiftWheelsAction] valid options are: lowered raised')
                success = False

        self._result.success = success
        if success:
            rospy.loginfo('[MoveShiftWheelsAction] succeeded')
            self._action_server.set_succeeded(self._result)
        else:
            rospy.loginfo('[MoveShiftWheelsAction] aborted')
            self._action_server.set_aborted(self._result)

    def _preempt_callback(self):
        rospy.loginfo('[MoveShiftWheelsAction] preempted')


def main():
    try:
        base_controller_node = BaseControllerNode('hardware_sim')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
