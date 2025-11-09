# ROS2 Imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Other Imports
import sys
# sys.path.append('../lib/python/arm64')
import robot_interface as sdk

# --- Global Settings ---
HIGHLEVEL = 0xee
# --- CHECK YOUR IP AND PORTS ---
# ROBOT_IP = "192.168.123.161"  # Network cable
ROBOT_IP = "192.168.12.1"   # Wi-Fi
ROBOT_PORT = 8082
LOCAL_PORT = 8080

class CommandExecutor(Node):

    def __init__(self):
        super().__init__("execute_twist")
        self.twistSub = self.create_subscription(Twist, 'go1_twist', self.moveRobot, 10)
        self.get_logger().info('Executing twist !')

        # TRY CONNECTING TO ROBOT
        try:
            self.udp = sdk.UDP(HIGHLEVEL, LOCAL_PORT, ROBOT_IP, ROBOT_PORT)
            self.cmd2go = sdk.HighCmd()
            self.state = sdk.HighState()
            self.udp.InitCmdData(self.cmd2go)
            self.get_logger().info(f"Connected to GO1 : {self.udp}")
        
        except Exception as e:
            self.get_logger().info(f"Couldn't connect to robot : {e}")
        
        
    def moveRobot(self):
        

def main(args=None):
    rclpy.init(args=args)
    executor = CommandExecutor()
    rclpy.spin(executor)
    executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        

        
        
    

