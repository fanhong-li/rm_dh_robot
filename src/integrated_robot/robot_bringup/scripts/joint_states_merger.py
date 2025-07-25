#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateMerger(Node):
    def __init__(self):
        super().__init__('joint_state_merger')
        self.arm_state = None
        self.gripper_state = None
        # 发布合并后的joint_states
        self.merged_publisher = self.create_publisher(
            JointState, '/robot_state_publisher/joint_states', 10
        )
        self.create_subscription(JointState, '/joint_states', self.arm_cb, 10)
        self.create_subscription(JointState, '/gripper/joint_states', self.gripper_cb, 10)

    def arm_cb(self, msg):
        self.arm_state = msg
        self.publish_merged()

    def gripper_cb(self, msg):
        self.gripper_state = msg
        self.publish_merged()

    def publish_merged(self):
        if self.arm_state is None or self.gripper_state is None:
            return
        merged = JointState()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.name = list(self.arm_state.name) + list(self.gripper_state.name)
        merged.position = list(self.arm_state.position) + list(self.gripper_state.position)
        merged.velocity = list(self.arm_state.velocity) + list(self.gripper_state.velocity)
        merged.effort = list(self.arm_state.effort) + list(self.gripper_state.effort)
        self.merged_publisher.publish(merged)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateMerger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main() 