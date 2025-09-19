import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import the Node class for creating ROS 2 nodes
from rclpy.action import ActionClient  # Import ActionClient for interacting with action servers
from example_interfaces.action import Fibonacci  # Import the Fibonacci action definition
from rclpy.logging import get_logger  # Import get_logger for logging



class FibClient(Node):  # Define a class that inherits from Node
    def __init__(self):  # Constructor for the FibClient class
        super().__init__('fibonacci_action_client')  # Initialize the node with a name
        self.get_logger().info("Initializing Fibonacci Action Client node")
        self.cli = ActionClient(self, Fibonacci, 'fibonacci')  # Create an ActionClient for the Fibonacci action
        self.get_logger().info("ActionClient created for 'fibonacci' action")
        self.send_goal(10)  # Send a goal with order 10 as a demo

    def send_goal(self, order: int):  # Method to send a goal to the action server
        self.get_logger().info(f"Waiting for action server with timeout 5.0s...")
        if not self.cli.wait_for_server(timeout_sec=5.0):  # Wait for the action server to be available
            self.get_logger().error("Action server not available")  # Log error if server is unavailable
            return  # Exit the method
        self.get_logger().info(f"Action server available. Sending goal with order: {order}")
        goal = Fibonacci.Goal()  # Create a new goal message
        goal.order = order  # Set the order field of the goal

        # Send the goal asynchronously with feedback callback
        self.cli.send_goal_async(goal, feedback_callback=self.fb_cb)\
            .add_done_callback(self.goal_resp)  # Add a callback for when the goal is accepted/rejected
        self.get_logger().info("Goal sent asynchronously")

    def fb_cb(self, fb):  # Callback for receiving feedback from the action server
        self.get_logger().info(f"Feedback received: {list(fb.feedback.sequence)}")  # Log the feedback sequence

    def goal_resp(self, fut):  # Callback for goal response (accepted/rejected)
        self.get_logger().info("Goal response received")
        handle = fut.result()  # Get the goal handle from the future
        if not handle.accepted:  # Check if the goal was accepted
            self.get_logger().error("Goal rejected")  # Log error if goal was rejected
            return  # Exit the method
        self.get_logger().info("Goal accepted by server, waiting for result...")
        handle.get_result_async().add_done_callback(self.done)  # Request the result asynchronously and set callback

    def done(self, fut):  # Callback for when the result is available
        self.get_logger().info("Result received from action server")
        result = fut.result().result  # Get the result from the future
        self.get_logger().info(f"Result: {list(result.sequence)}")  # Log the final result sequence

def main():  # Main function to start the node
    rclpy.init()  # Initialize the ROS 2 Python client library
    get_logger("main").info("Starting Fibonacci Action Client node")
    rclpy.spin(FibClient())  # Create and spin the FibClient node
    get_logger("main").info("Shutting down Fibonacci Action Client node")
    rclpy.shutdown()  # Shutdown the ROS 2 client library
