import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('vel_publisher')
        self.publisher_ = self.create_publisher(Float64, 'vel_topic', 10)
        self.timer = self.create_timer(1.0/30.0, self.publish_float)
        self.get_logger().info('Vel Publisher Node Started')

    def publish_float(self):
        msg = Float64()
        msg.data = 1.2  # m/s
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
