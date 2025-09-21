# advanced_comms_demo/qos_talker.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

class QosTalker(Node):
    def __init__(self):
        super().__init__('qos_talker')

        # Define a RELIABLE QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(
            String,
            'chatter',
            qos_profile) # Apply the QoS profile here

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0
        self.get_logger().info('QoS Talker started. Publishing with RELIABLE durability.')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World (Reliable): {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = QosTalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()