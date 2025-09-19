import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for ROS 2 nodes
from geometry_msgs.msg import Twist  # Message type for velocity commands

class CircleDriver(Node):
    def __init__(self):
        super().__init__('circle_driver')  # Initialize the node with the name 'circle_driver'
        self.get_logger().info('CircleDriver node has been started.')
        # Create a publisher to send Twist messages to the turtle's velocity topic
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # Create a timer to call the tick method every 0.1 seconds (10 Hz)
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        msg = Twist()  # Create a new Twist message
        msg.linear.x = 1.0     # Set forward linear velocity
        msg.angular.z = 1.0    # Set angular velocity for turning
        self.pub.publish(msg)  # Publish the velocity command
        self.get_logger().debug(f'Published Twist: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

def main():
    rclpy.init()  # Initialize ROS 2 Python client library
    node = CircleDriver()  # Create an instance of the CircleDriver node
    node.get_logger().info('Spinning node...')
    rclpy.spin(node)  # Keep the node running, processing callbacks
    node.get_logger().info('Shutting down node...')
    node.destroy_node()  # Clean up the node after exiting
    rclpy.shutdown()  # Shutdown the ROS 2 client library

if __name__ == '__main__':
    main()  # Run the main function if this file is executed directly
