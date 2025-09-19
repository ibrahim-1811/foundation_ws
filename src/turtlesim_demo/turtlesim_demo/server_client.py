import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen
from std_srvs.srv import Empty
from rclpy.logging import get_logger  # Import get_logger for logging

class PenAndClearClient(Node):
    def __init__(self):
        super().__init__('pen_and_clear_client')
        self.get_logger().info("Initializing PenAndClearClient node.")
        self.pen = self.create_client(SetPen, '/turtle1/set_pen')
        self.get_logger().info("Created SetPen client for /turtle1/set_pen.")
        self.clear = self.create_client(Empty, '/clear')
        self.get_logger().info("Created Empty client for /clear.")
        self.create_timer(0.5, self.run_once)
        self.done = False

    def run_once(self):
        if self.done:
            self.get_logger().debug("run_once called but already done.")
            return
        self.done = True
        self.get_logger().info("Running service calls for SetPen and Clear.")

        for cli, name in [(self.pen, '/turtle1/set_pen'), (self.clear, '/clear')]:
            self.get_logger().info(f"Waiting for service {name}...")
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f"Service {name} not available")
                rclpy.shutdown()
                return
            self.get_logger().info(f"Service {name} is available.")

        req = SetPen.Request(r=180, g=0, b=255, width=4, off=0)
        self.get_logger().info("Calling SetPen service with r=180, g=0, b=255, width=4, off=0.")
        self.pen.call_async(req).add_done_callback(self.after_pen)

    def after_pen(self, future):
        if future.result() is not None:
            self.get_logger().info("SetPen service call succeeded.")
        else:
            self.get_logger().error("SetPen service call failed.")
        self.get_logger().info("Calling Clear service.")
        clear_future = self.clear.call_async(Empty.Request())
        clear_future.add_done_callback(self.after_clear)

    def after_clear(self, future):
        self.get_logger().info("Set pen (purple, w=4) & cleared background.")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = PenAndClearClient()
    rclpy.spin(node)
