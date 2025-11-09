import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Or a custom message type for more complex commands
from pynput import keyboard # Or another library like curses or prompt_toolkit

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_cmd_publisher')
        self.publisher_ = self.create_publisher(String, 'keyboard_cmd', 10)
        self.get_logger().info('Keyboard Publisher node started.')

        # Initialize keyboard listener
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()

    def on_press(self, key):
        try:
            command = f'Key pressed: {key.char}'
            
        except AttributeError:
            command = f'Special key pressed: {key}'
        
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Current Command: "{msg.data}"')
        self.get_logger().info("-----------------------------------------------------------------------------------------------------")
        self.get_logger().info("[W] Fwd | [S] Stop | [X] Back | [G] Damping | [Q] Quit | [K] Rotate | [H] Stand Up | [Y] Listen2Twist")

    def on_release(self, key):
        if key == keyboard.Key.esc:
            # Stop listener
            return False

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.listener.stop() # Ensure the keyboard listener is stopped
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()