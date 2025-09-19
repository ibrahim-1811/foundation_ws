import time  # Import time module for sleep
import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Node base class
from rclpy.action import ActionServer  # ActionServer class for actions
from example_interfaces.action import Fibonacci  # Fibonacci action definition

class FibServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')  # Initialize node with name
        self.get_logger().info('Initializing Fibonacci Action Server...')  # Log initialization
        # Create an ActionServer for the Fibonacci action
        self._srv = ActionServer(self, Fibonacci, 'fibonacci', self.execute)

    async def execute(self, goal_handle):
        # Get the requested order, ensure at least 2
        order = max(2, goal_handle.request.order)
        self.get_logger().info(f'Received goal request: order={order}')  # Log received goal
        seq = [0, 1]  # Initialize Fibonacci sequence
        fb = Fibonacci.Feedback()  # Create feedback message

        for i in range(2, order):
            # Check if cancel was requested
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled by client.')  # Log cancellation
                goal_handle.canceled()
                return Fibonacci.Result()
            # Calculate next Fibonacci number and append
            seq.append(seq[-1] + seq[-2])
            fb.sequence = seq  # Update feedback
            goal_handle.publish_feedback(fb)  # Publish feedback
            self.get_logger().info(f'Published feedback: {seq}')  # Log feedback
            time.sleep(0.2)  # Sleep for demonstration

        goal_handle.succeed()  # Mark goal as succeeded
        self.get_logger().info(f'Goal succeeded. Result: {seq[:order]}')  # Log result
        res = Fibonacci.Result()  # Create result message
        res.sequence = seq[:order]  # Set result sequence
        return res  # Return result

def main():
    rclpy.init()  # Initialize ROS 2 Python client library
    rclpy.spin(FibServer())  # Spin node to process callbacks
    rclpy.shutdown()  # Shutdown ROS 2

