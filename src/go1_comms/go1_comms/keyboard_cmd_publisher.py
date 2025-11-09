import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import termios
import tty

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_cmd_publisher')
        self.publisher_ = self.create_publisher(String, 'keyboard_cmd', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info('Keyboard Publisher node started. Press keys to publish.')
        self.print_instructions()
        
        self.timer = self.create_timer(0.1, self.read_key_callback)

    def print_instructions(self):
        print("--------------------------------------------------")
        print("[W] Fwd | [S] Stop | [X] Back")
        print("[G] Damping | [K] Rotate | [H] Stand Up")
        print("[Y] Listen2Twist | [Q] Quit")
        print("--------------------------------------------------")
        print("Press Ctrl+C to exit cleanly.")

    def get_key(self):
        # Switch terminal to raw mode to read single characters without pressing Enter
        tty.setraw(sys.stdin.fileno())
        # Non-blocking check if data is available on stdin
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        # Restore standard terminal settings immediately
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def read_key_callback(self):
        key = self.get_key()
        if key:
            # Handle Ctrl+C (ASCII 3) explicitly if needed, though KeyboardInterrupt usually catches it.
            if key == '\x03' or key.lower() == 'q': 
                raise KeyboardInterrupt
            
            msg = String()
            msg.data = key
            self.publisher_.publish(msg)
            # Use print instead of get_logger so it doesn't clutter the raw terminal view too much
            print(f'\rPublished: {key}   ', end='') 

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        # Restore terminal settings on exit, or the terminal will be messed up
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\nNode stopped and terminal settings restored.")

if __name__ == '__main__':
    main()