#!/usr/bin/env python3
import argparse
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Bool, Float64MultiArray, String

from dingo_control.Command import Command
from dingo_control.Config import Configuration, Leg_linkage
from dingo_control.Controller import Controller
from dingo_control.Kinematics import four_legs_inverse_kinematics
from dingo_control.State import BehaviorState, RobotMode, State
from dingo_control_msgs.msg import JointSpace, TaskSpace


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
        self._loop_period = 1.0 / self.message_rate

        self.is_sim = is_sim
        self.is_physical = is_physical
        self.use_imu = use_imu

        # ✅ MODE ALIAS MAP (THIS WAS YOUR ISSUE AREA)
        self._mode_map = {
            'rest': RobotMode.REST,
            'stand': RobotMode.REST,   # compatibility
            'trot': RobotMode.TROT,
            'walk': RobotMode.TROT,    # compatibility
            'sit': RobotMode.SIT,
            'lay': RobotMode.LAY,
        }

        self.latest_cmd_vel = Twist()
        self.latest_mode = RobotMode.REST

        self.desired_mode_topic = '/dingo_mode'
        self.filtered_cmd_vel_topic = '/cmd_vel'

        self.current_mode_topic = node.declare_parameter(
            'current_mode_topic', '/dingo/current_mode'
        ).value

        self._current_mode_pub = node.create_publisher(String, self.current_mode_topic, 10)

        self._declare_behavior_pose_parameters()

        # ---------------- SUBS ----------------
        self.mode_sub = node.create_subscription(
            String, self.desired_mode_topic, self.update_robot_mode, 10
        )

        self.cmd_vel_sub = node.create_subscription(
            Twist, self.filtered_cmd_vel_topic, self.update_cmd_vel, 10
        )

        self.estop_status_sub = node.create_subscription(
            Bool, '/emergency_stop_status', self.update_emergency_stop_status, 10
        )

        # ---------------- SIM OUTPUT ----------------
        self._sim_leg_cmds_pub = None
        if self.is_sim:
            # 👉 SEND TROT OUTPUT TO RAW TOPIC (mode manager will take over final output)
            gz_leg_topic = '/dingo/trot_joint_commands'

            self._sim_leg_cmds_pub = node.create_publisher(
                Float64MultiArray, gz_leg_topic, 10
            )

        # ---------------- CONFIG ----------------
        self.config = Configuration()
        self._apply_behavior_pose_parameters()

                # REST height slider mapping
        # Slider input is expected to be:
        #   -1.0 = bottom
        #    0.0 = center
        #   +1.0 = top
        self.rest_height_center = node.declare_parameter(
            'rest_height_center',
            self.config.default_z_ref
        ).value

        self.rest_height_min = node.declare_parameter(
            'rest_height_min',
            self.rest_height_center - 0.02
        ).value

        self.rest_height_max = node.declare_parameter(
            'rest_height_max',
            self.rest_height_center + 0.02
        ).value

        # ---------------- CONTROLLER ----------------
        self.controller = Controller(self.config, four_legs_inverse_kinematics, node)
        self.state = State()

        self.state.robot_mode = RobotMode.REST
        self.state.behavior_state = BehaviorState.REST
        
                # Use the already-stable startup height as the slider center
        self.rest_height_center = float(self.state.height)
        self.rest_height_min = self.rest_height_center + 0.08
        self.rest_height_max = self.rest_height_center - 0.04


        self.node.get_logger().info("✅ Dingo mode driver ready")

        self.publish_current_mode()

    # ---------------- PARAMETERS ----------------

    def _declare_behavior_pose_parameters(self):
        defaults = {
            'sit_x_offsets': [-0.03, -0.03, 0.09, 0.09],
            'sit_y_offsets': [0.0, 0.0, 0.0, 0.0],
            'sit_z_offsets': [-0.18, -0.18, -0.18, -0.18],
            'lay_x_offsets': [-0.015, -0.015, 0.035, 0.035],
            'lay_y_offsets': [0.0, 0.0, 0.0, 0.0],
            'lay_z_offsets': [-0.24, -0.24, -0.24, -0.24],
        }

        for name, default in defaults.items():
            self.node.declare_parameter(name, default)

    def _param_vec(self, name):
        return np.array(self.node.get_parameter(name).value, dtype=float)

    def _apply_behavior_pose_parameters(self):
        self.config.set_behavior_pose_offsets(
            sit_x=self._param_vec('sit_x_offsets'),
            sit_y=self._param_vec('sit_y_offsets'),
            sit_z=self._param_vec('sit_z_offsets'),
            lay_x=self._param_vec('lay_x_offsets'),
            lay_y=self._param_vec('lay_y_offsets'),
            lay_z=self._param_vec('lay_z_offsets'),
        )

    # ---------------- CALLBACKS ----------------

    def update_robot_mode(self, msg: String):
        mode = msg.data.strip().lower()

        if mode not in self._mode_map:
            self.node.get_logger().warn(f"Unknown mode: {mode}")
            return

        self.latest_mode = self._mode_map[mode]
        self.node.get_logger().info(f"Mode -> {self.latest_mode.value}")
        self.publish_current_mode()

    def publish_current_mode(self):
        msg = String()
        msg.data = self.latest_mode.value
        self._current_mode_pub.publish(msg)

    def update_cmd_vel(self, msg: Twist):
        self.latest_cmd_vel = msg

    def update_emergency_stop_status(self, msg):
        self.state.currently_estopped = 1 if msg.data else 0

    # ---------------- CORE ----------------

    def build_command(self):
        command = Command()

        command.horizontal_velocity = np.array([
            self.latest_cmd_vel.linear.x,
            self.latest_cmd_vel.linear.y
        ])
        command.yaw_rate = self.latest_cmd_vel.angular.z

        # Preserve current state by default
        command.height = self.state.height
        command.pitch = self.state.pitch
        command.roll = self.state.roll

        # Height slider is normalized:
        #   -1.0 = bottom
        #    0.0 = center
        #   +1.0 = top
        #
        # Use it in BOTH REST and TROT so gait can adapt like old Dingo did.
        if self.latest_mode in (RobotMode.REST, RobotMode.TROT):
            slider = float(np.clip(self.latest_cmd_vel.linear.z, -1.0, 1.0))

            if slider >= 0.0:
                command.height = self.rest_height_center + slider * (
                    self.rest_height_max - self.rest_height_center
                )
            else:
                command.height = self.rest_height_center + (-slider) * (
                    self.rest_height_min - self.rest_height_center
                )

        # Optional: keep pitch/roll only in REST for now
        if self.latest_mode == RobotMode.REST:
            command.roll = np.clip(self.latest_cmd_vel.angular.x, -1.0, 1.0) * 0.10
            command.pitch = np.clip(self.latest_cmd_vel.angular.y, -1.0, 1.0) * 0.10

        return command

    def apply_mode(self):
        if self.latest_mode == RobotMode.TROT:
            self.state.behavior_state = BehaviorState.TROT
        elif self.latest_mode == RobotMode.REST:
            self.state.behavior_state = BehaviorState.REST
        elif self.latest_mode == RobotMode.SIT:
            self.state.behavior_state = BehaviorState.SIT
        elif self.latest_mode == RobotMode.LAY:
            self.state.behavior_state = BehaviorState.LAY

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)

            self.apply_mode()

            command = self.build_command()

            self.controller.run(self.state, command)

            if self.is_sim:
                self.publish_joints(self.state.joint_angles)

            time.sleep(self._loop_period)

    def publish_joints(self, joint_angles):
        if not self._sim_leg_cmds_pub:
            return

        msg = Float64MultiArray()

        msg.data = [
            float(joint_angles[0, 0]), float(joint_angles[1, 0]), float(joint_angles[2, 0]),
            float(joint_angles[0, 1]), float(joint_angles[1, 1]), float(joint_angles[2, 1]),
            float(joint_angles[0, 2]), float(joint_angles[1, 2]), float(joint_angles[2, 2]),
            float(joint_angles[0, 3]), float(joint_angles[1, 3]), float(joint_angles[2, 3]),
        ]

        self._sim_leg_cmds_pub.publish(msg)


def main(args=None):
    import sys
    argv = sys.argv if args is None else args
    parsed = parse_driver_args(argv)

    rclpy.init(args=args)
    node = rclpy.create_node('dingo_driver')

    try:
        driver = DingoDriver(parsed.is_sim, parsed.is_physical, parsed.use_imu, node)
        driver.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()