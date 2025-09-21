import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String


class AdvanceCommDemoNode(Node):
    def __init__(self):
        super().__init__('advance_comm_demo_node')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.KEEP_LAST,
            depth=10
        )

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.publisher = self.create_publisher(String, 'advance_topic', qos_profile)
        self.i = 0
        self.get_logger().info("AdvanceCommDemoNode has been started.")



    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, this is message {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = AdvanceCommDemoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()  