#!/usr/bin/env python3
import argparse
import platform
import signal
import sys
import time

import numpy as np
import rclpy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float64

from dingo_control.Config import Configuration, Leg_linkage
from dingo_control.Controller import Controller
from dingo_control.Kinematics import four_legs_inverse_kinematics
from dingo_control.State import State
from dingo_input_interfacing.InputInterface import InputController, InputInterface


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('is_sim', nargs='?', type=int, default=0)
    parser.add_argument('is_physical', nargs='?', type=int, default=0)
    return parser.parse_args(remove_ros_args(argv)[1:])


def signal_handler(sig, frame):
    sys.exit(0)


def main(use_imu=False):
    args = parse_args(sys.argv)
    is_sim = args.is_sim
    is_physical = args.is_physical

    rclpy.init()
    node = rclpy.create_node('dingo')
    message_rate = 50
    rate = node.create_rate(message_rate)

    signal.signal(signal.SIGINT, signal_handler)

    publishers = []
    if is_sim:
        command_topics = [
            '/notspot_controller/FR1_joint/command',
            '/notspot_controller/FR2_joint/command',
            '/notspot_controller/FR3_joint/command',
            '/notspot_controller/FL1_joint/command',
            '/notspot_controller/FL2_joint/command',
            '/notspot_controller/FL3_joint/command',
            '/notspot_controller/RR1_joint/command',
            '/notspot_controller/RR2_joint/command',
            '/notspot_controller/RR3_joint/command',
            '/notspot_controller/RL1_joint/command',
            '/notspot_controller/RL2_joint/command',
            '/notspot_controller/RL3_joint/command',
        ]
        for topic in command_topics:
            publishers.append(node.create_publisher(Float64, topic, 1))

    config = Configuration()
    hardware_interface = None
    imu = None
    if is_physical:
        from dingo_servo_interfacing.HardwareInterface import HardwareInterface

        linkage = Leg_linkage(config)
        hardware_interface = HardwareInterface(linkage)
    if use_imu:
        from dingo_peripheral_interfacing.IMU import IMU

        imu = IMU()

    controller = Controller(
        config,
        four_legs_inverse_kinematics,
        node,
    )
    state = State()
    print('Creating input listener...')
    input_interface = InputInterface(config, node)
    print(platform.processor())
    InputController(1, platform.processor())
    print('Done.')

    print('Summary of gait parameters:')
    print('overlap time: ', config.overlap_time)
    print('swing time: ', config.swing_time)
    print('z clearance: ', config.z_clearance)
    print('x shift: ', config.x_shift)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            print('Waiting for L1 to activate robot.')

            while rclpy.ok():
                command = input_interface.get_command(state, message_rate)
                if command.joystick_control_event == 1:
                    break
                rate.sleep()
                rclpy.spin_once(node, timeout_sec=0.0)
            print('Robot activated.')

            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.0)
                t_start = time.perf_counter()
                command = input_interface.get_command(state, message_rate)
                if command.joystick_control_event == 1:
                    print('Deactivating Robot')
                    break

                quat_orientation = (
                    imu.read_orientation() if use_imu and imu is not None else np.array([1, 0, 0, 0])
                )
                state.quat_orientation = quat_orientation

                controller.run(state, command)

                if is_sim:
                    rows, cols = state.joint_angles.shape
                    idx = 0
                    for col in range(cols):
                        for row in range(rows):
                            publishers[idx].publish(Float64(data=float(state.joint_angles[row, col])))
                            idx += 1

                if is_physical:
                    hardware_interface.set_actuator_postions(state.joint_angles)

                time.perf_counter() - t_start
                rate.sleep()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
