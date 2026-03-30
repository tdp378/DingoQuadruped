#!/usr/bin/env python3
import argparse
import signal
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float64MultiArray

from dingo_control.Config import Configuration, Leg_linkage
from dingo_control.Controller import Controller
from dingo_control.Kinematics import four_legs_inverse_kinematics
from dingo_control.State import BehaviorState, State
from dingo_control_msgs.msg import JointSpace, TaskSpace
from dingo_input_interfacing.InputInterface import InputInterface


def parse_driver_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('is_sim', nargs='?', type=int, default=0)
    parser.add_argument('is_physical', nargs='?', type=int, default=1)
    parser.add_argument('use_imu', nargs='?', type=int, default=0)
    return parser.parse_args(remove_ros_args(argv)[1:])


class DingoDriver:
    def __init__(self, is_sim, is_physical, use_imu, node: Node):
        self.node = node
        self.message_rate = 50
        # Wall-clock sleep, not node.create_rate(): Rate.sleep() deadlocks under
        # use_sim_time because the timer callback needs spin_once() to fire, but
        # we're blocked in sleep() and not spinning.
        self._loop_period = 1.0 / self.message_rate

        self.is_sim = is_sim
        self.is_physical = is_physical
        self.use_imu = use_imu

        self.joint_command_sub = node.create_subscription(
            JointSpace, '/joint_space_cmd', self.run_joint_space_command, 10
        )
        self.task_command_sub = node.create_subscription(
            TaskSpace, '/task_space_cmd', self.run_task_space_command, 10
        )
        self.estop_status_sub = node.create_subscription(
            Bool, '/emergency_stop_status', self.update_emergency_stop_status, 10
        )
        self.external_commands_enabled = 0

        # Gazebo Sim (gz_ros2_control): JointGroupPositionController on /leg_joint_position_controller/commands
        self._sim_leg_cmds_pub = None
        if self.is_sim:
            gz_leg_topic = node.declare_parameter(
                'gz_leg_command_topic', '/leg_joint_position_controller/commands'
            ).value
            self._sim_leg_cmds_pub = node.create_publisher(Float64MultiArray, gz_leg_topic, 10)

        self.config = Configuration()
        self.hardware_interface = None
        if is_physical:
            from dingo_servo_interfacing.HardwareInterface import HardwareInterface

            self.linkage = Leg_linkage(self.config)
            self.hardware_interface = HardwareInterface(self.linkage)
        self.imu = None
        if self.use_imu:
            from dingo_peripheral_interfacing.IMU import IMU

            self.imu = IMU()

        self.controller = Controller(
            self.config,
            four_legs_inverse_kinematics,
            node,
        )

        self.state = State()
        log = node.get_logger()
        log.info('Creating input listener...')
        self.input_interface = InputInterface(self.config, node)
        log.info('Input listener successfully initialised... Robot will now receive commands via Joy messages')

        log.info('Summary of current gait parameters:')
        log.info('overlap time: %.2f' % self.config.overlap_time)
        log.info('swing time: %.2f' % self.config.swing_time)
        log.info('z clearance: %.2f' % self.config.z_clearance)
        log.info('back leg x shift: %.2f' % self.config.rear_leg_x_shift)
        log.info('front leg x shift: %.2f' % self.config.front_leg_x_shift)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)
            if self.state.currently_estopped == 1:
                self.node.get_logger().warn(
                    'E-stop pressed. Controlling code now disabled until E-stop is released'
                )
                self.state.trotting_active = 0
                while self.state.currently_estopped == 1:
                    rclpy.spin_once(self.node, timeout_sec=0.0)
                    time.sleep(self._loop_period)
                self.node.get_logger().info('E-stop released')

            self.node.get_logger().info('Manual robot control active. Currently not accepting external commands')
            command = self.input_interface.get_command(self.state, self.message_rate)
            self.state.behavior_state = BehaviorState.REST
            self.controller.run(self.state, command)
            self.controller.publish_joint_space_command(self.state.joint_angles)
            self.controller.publish_task_space_command(self.state.rotated_foot_locations)
            if self.is_sim:
                self.publish_joints_to_sim(self.state.joint_angles)
            if self.is_physical:
                self.hardware_interface.set_actuator_postions(self.state.joint_angles)
            self._loop_count = 0
            while self.state.currently_estopped == 0:
                rclpy.spin_once(self.node, timeout_sec=0.0)
                t_start = time.perf_counter()

                command = self.input_interface.get_command(self.state, self.message_rate)

                self._loop_count += 1
                if self._loop_count % 250 == 1:
                    self.node.get_logger().info(
                        f'loop #{self._loop_count}  state={self.state.behavior_state}  '
                        f'vel={command.horizontal_velocity}  '
                        f'sim_pub={self._sim_leg_cmds_pub is not None}'
                    )

                if command.joystick_control_event == 1:
                    if self.state.currently_estopped == 0:
                        self.external_commands_enabled = 1
                        break
                    self.node.get_logger().error(
                        'Received Request to enable external control, but e-stop is pressed so the request '
                        'has been ignored. Please release e-stop and try again'
                    )

                self.state.euler_orientation = (
                    self.imu.read_orientation() if self.use_imu else np.array([0, 0, 0])
                )
                self.controller.run(self.state, command)

                if self.state.behavior_state in (BehaviorState.TROT, BehaviorState.REST):
                    self.controller.publish_joint_space_command(self.state.joint_angles)
                    self.controller.publish_task_space_command(self.state.rotated_foot_locations)
                    if self.is_sim:
                        self.publish_joints_to_sim(self.state.joint_angles)
                    if self.is_physical:
                        self.hardware_interface.set_actuator_postions(self.state.joint_angles)
                    time.perf_counter() - t_start
                else:
                    if self.is_sim:
                        self.publish_joints_to_sim(self.state.joint_angles)
                time.sleep(self._loop_period)

            if self.state.currently_estopped == 0:
                self.node.get_logger().info('Manual Control deactivated. Now accepting external commands')
                command = self.input_interface.get_command(self.state, self.message_rate)
                self.state.behavior_state = BehaviorState.REST
                self.controller.run(self.state, command)
                self.controller.publish_joint_space_command(self.state.joint_angles)
                self.controller.publish_task_space_command(self.state.rotated_foot_locations)
                if self.is_sim:
                    self.publish_joints_to_sim(self.state.joint_angles)
                if self.is_physical:
                    self.hardware_interface.set_actuator_postions(self.state.joint_angles)
                while self.state.currently_estopped == 0:
                    rclpy.spin_once(self.node, timeout_sec=0.0)
                    command = self.input_interface.get_command(self.state, self.message_rate)
                    if command.joystick_control_event == 1:
                        self.external_commands_enabled = 0
                        break
                    time.sleep(self._loop_period)

    def update_emergency_stop_status(self, msg):
        if msg.data:
            self.state.currently_estopped = 1
        else:
            self.state.currently_estopped = 0

    def run_task_space_command(self, msg):
        if self.external_commands_enabled == 1 and self.state.currently_estopped == 0:
            foot_locations = np.zeros((3, 4))
            foot_locations[:, 0] = [msg.fr_foot.x, msg.fr_foot.y, msg.fr_foot.z]
            foot_locations[:, 1] = [msg.fl_foot.x, msg.fl_foot.y, msg.fl_foot.z]
            foot_locations[:, 2] = [msg.rr_foot.x, msg.rr_foot.y, msg.rr_foot.z]
            foot_locations[:, 3] = [msg.rl_foot.x, msg.rl_foot.y, msg.rl_foot.z]
            print(foot_locations)
            joint_angles = self.controller.inverse_kinematics(foot_locations, self.config)
            if self.is_sim:
                self.publish_joints_to_sim(joint_angles)

            if self.is_physical:
                self.hardware_interface.set_actuator_postions(joint_angles)

        elif self.external_commands_enabled == 0:
            self.node.get_logger().error(
                'ERROR: Robot not accepting commands. Please deactivate manual control before sending control commands'
            )
        elif self.state.currently_estopped == 1:
            self.node.get_logger().error('ERROR: Robot currently estopped. Please release before trying to send commands')

    def run_joint_space_command(self, msg):
        if self.external_commands_enabled == 1 and self.state.currently_estopped == 0:
            joint_angles = np.zeros((3, 4))
            joint_angles[:, 0] = [
                np.radians(msg.fr_foot.theta1),
                np.radians(msg.fr_foot.theta2),
                np.radians(msg.fr_foot.theta3),
            ]
            joint_angles[:, 1] = [
                np.radians(msg.fl_foot.theta1),
                np.radians(msg.fl_foot.theta2),
                np.radians(msg.fl_foot.theta3),
            ]
            joint_angles[:, 2] = [
                np.radians(msg.rr_foot.theta1),
                np.radians(msg.rr_foot.theta2),
                np.radians(msg.rr_foot.theta3),
            ]
            joint_angles[:, 3] = [
                np.radians(msg.rl_foot.theta1),
                np.radians(msg.rl_foot.theta2),
                np.radians(msg.rl_foot.theta3),
            ]
            print(joint_angles)

            if self.is_sim:
                self.publish_joints_to_sim(joint_angles)

            if self.is_physical:
                self.hardware_interface.set_actuator_postions(joint_angles)

        elif self.external_commands_enabled == 0:
            self.node.get_logger().error(
                'ERROR: Robot not accepting commands. Please deactivate manual control before sending control commands'
            )
        elif self.state.currently_estopped == 1:
            self.node.get_logger().error('ERROR: Robot currently estopped. Please release before trying to send commands')

    def publish_joints_to_sim(self, joint_angles):
        # Order matches dingo_gz_ros2_control.yaml: FR, FL, RR, RL × (theta1, theta2, theta3); radians
        rows, cols = joint_angles.shape
        msg = Float64MultiArray()
        msg.data = [
            float(joint_angles[row, col])
            for col in range(cols)
            for row in range(rows)
        ]
        self._sim_leg_cmds_pub.publish(msg)


def signal_handler(sig, frame):
    sys.exit(0)


def main():
    args = parse_driver_args(sys.argv)
    rclpy.init()
    node = rclpy.create_node('dingo_driver')
    # use_sim_time is already declared by rclpy (Jazzy+); only override for sim.
    if args.is_sim:
        node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    signal.signal(signal.SIGINT, signal_handler)
    driver = DingoDriver(args.is_sim, args.is_physical, args.use_imu, node)
    try:
        driver.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
