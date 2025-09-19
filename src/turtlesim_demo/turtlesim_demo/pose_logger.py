import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import the Node class from rclpy
from turtlesim.msg import Pose  # Import the Pose message type from turtlesim

class PoseLogger(Node):  # Define a class PoseLogger that inherits from Node
    def __init__(self):  # Constructor for the PoseLogger class
        super().__init__('pose_logger')  # Initialize the Node with the name 'pose_logger'
        self.get_logger().info("Initializing PoseLogger node.")  # Logger statement
        self.create_subscription(Pose, '/turtle1/pose', self.cb, 10)  # Create a subscription to the '/turtle1/pose' topic with Pose message type, using callback 'cb', and queue size 10
        self.get_logger().info("Subscription to /turtle1/pose created.")  # Logger statement

    def cb(self, msg: Pose):  # Callback function that is called when a new Pose message is received
        self.get_logger().debug(f"Received Pose message: {msg}")  # Logger statement
        self.get_logger().info(f"x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}")  # Log the x, y, and theta values from the Pose message with two decimal places

def main():  # Main function to start the node
    rclpy.init()  # Initialize the ROS 2 Python client library
    rclpy.logging.get_logger("pose_logger_main").info("ROS 2 Python client library initialized.")  # Logger statement
    node = PoseLogger()  # Create an instance of the PoseLogger node
    rclpy.logging.get_logger("pose_logger_main").info("Spinning PoseLogger node.")  # Logger statement
    rclpy.spin(node)  # Keep the node running, processing callbacks
    node.get_logger().info("Shutting down PoseLogger node.")  # Logger statement
    node.destroy_node()  # Destroy the node explicitly (cleanup)
    rclpy.shutdown()  # Shutdown the ROS 2 Python client library
    rclpy.logging.get_logger("pose_logger_main").info("ROS 2 Python client library shutdown.")  # Logger statement

if __name__ == '__main__':  # Check if the script is being run directly
    main()  # Call the main function
