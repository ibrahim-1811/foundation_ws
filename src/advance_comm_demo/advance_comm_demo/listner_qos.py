# advanced_comms_demo/qos_listener.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

class QosListener(Node):
    def __init__(self):
        super().__init__('qos_listener')

        # Define a BEST EFFORT QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            qos_profile) # Apply the incompatible QoS profile

        self.get_logger().info('QoS Listener started. Listening with BEST EFFORT durability.')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = QosListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()