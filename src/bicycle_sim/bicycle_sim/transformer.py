#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import TransformStamped

class Transformer(Node):
    def __init__(self):
        super().__init__('transformer')

        self.odom_subscriber_ = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.odom_timestamp = None
        self.br = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


    def broadcast_transform(self):
        if self.robot_pose is not None and self.odom_timestamp is not None:
            t = TransformStamped()
            t.header.stamp = self.odom_timestamp
            t.header.frame_id = 'world'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.robot_pose.position.x
            t.transform.translation.y = self.robot_pose.position.y
            t.transform.translation.z = self.robot_pose.position.z

            q = self.robot_pose.orientation
            t.transform.rotation.x = q.x
            t.transform.rotation.y = q.y
            t.transform.rotation.z = q.z
            t.transform.rotation.w = q.w

            self.br.sendTransform(t)
        else:
            return

    def odom_callback(self, msg):
        self.odom_timestamp = msg.header.stamp
        self.robot_pose = msg.pose.pose
        self.broadcast_transform()



def main(args=None):
    rclpy.init(args=args)
    transformer = Transformer()
    rclpy.spin(transformer)
    transformer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()