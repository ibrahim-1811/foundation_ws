# advanced_comms_demo/lifecycle_talker.py
import rclpy
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from std_msgs.msg import String

class LifecycleTalker(Node):
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
        self.publisher_ = None
        self.timer = None
        self.count = 0
        self.get_logger().info(f'Node "{self.get_name()}" created.')

    # --- Transition Callbacks ---
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('In on_configure()')
        # In this state, we initialize resources.
        self.publisher_ = self.create_publisher(String, 'lifecycle_chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.timer.cancel() # Start the timer paused
        self.get_logger().info('Talker configured.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('In on_activate()')
        # In this state, we start the action.
        self.timer.reset()
        self.get_logger().info('Talker activated. Publishing will start.')
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('In on_deactivate()')
        # In this state, we stop the action.
        self.timer.cancel()
        self.get_logger().info('Talker deactivated. Publishing will stop.')
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('In on_cleanup()')
        # In this state, we destroy resources.
        self.destroy_publisher(self.publisher_)
        self.destroy_timer(self.timer)
        self.get_logger().info('Talker cleaned up.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('In on_shutdown()')
        self.destroy_publisher(self.publisher_)
        self.destroy_timer(self.timer)
        self.get_logger().info('Talker is shutting down.')
        return TransitionCallbackReturn.SUCCESS
    
    # --- The Main Action ---
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from lifecycle node: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleTalker('lifecycle_talker')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()