#!/usr/bin/env python3
"""Bridge-style node: mirrors legacy ROS 1 behavior for environments that expose gazebo_msgs."""
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import JointState


class JointStatesSubscriberPlugin(Node):
    def __init__(self):
        super().__init__('joint_states_subscriber_plugin')
        self.joint_positions = {}
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self._cli = self.create_client(SetModelState, '/gazebo/set_model_state')

    def joint_states_callback(self, msg):
        for joint_name, position in zip(msg.name, msg.position):
            self.joint_positions[joint_name] = position
        self.actuate_robot()

    def actuate_robot(self):
        if not self._cli.service_is_ready():
            return
        model_state = ModelState()
        model_state.model_name = 'your_robot_model_name'
        for _joint_name, position in self.joint_positions.items():
            model_state.pose.position.z = position
            model_state.twist.linear.x = 0.0
            model_state.twist.linear.y = 0.0
            model_state.twist.linear.z = 0.0
            model_state.twist.angular.x = 0.0
            model_state.twist.angular.y = 0.0
            model_state.twist.angular.z = 0.0
            model_state.reference_frame = 'world'
            model_state.pose.orientation.x = 0.0
            model_state.pose.orientation.y = 0.0
            model_state.pose.orientation.z = 0.0
            model_state.pose.orientation.w = 0.0
            req = SetModelState.Request()
            req.model_state = model_state
            self._cli.call_async(req)


def main():
    rclpy.init()
    node = JointStatesSubscriberPlugin()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
