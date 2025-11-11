# ROS2 Imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

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
        
        self.kcmdSub = self.create_subscription(String, 'keyboard_cmd', self.registerCommand, 10 )
        self.twistSub = self.create_subscription(Twist, 'go1_twist', self.try2moveRobot, 10)
        
        self.currentKey = 'H'
        self.isStanding = False

        # TRY CONNECTING TO ROBOT
        try:
            self.udp = sdk.UDP(HIGHLEVEL, LOCAL_PORT, ROBOT_IP, ROBOT_PORT)
            self.cmd2go = sdk.HighCmd()
            self.state = sdk.HighState()
            self.udp.InitCmdData(self.cmd2go)
            self.get_logger().info(f"Connected to GO1 : {self.udp}")

        except Exception as e:
            self.get_logger().info(f"Couldn't connect to robot : {e}")
                
    def registerCommand(self, msg):
        
        self.currentKey = msg.data
        self.get_logger().info(f"Command Registered : {self.currentKey}")


    def try2moveRobot(self, msg):
        
        if self.isStanding:
            x_vel = 0
            yaw_vel = 0
            self.cmd2go.mode = 2

            if self.currentKey == 'Y' or self.currentKey == 'y':
                x_vel = msg.linear.x
                yaw_vel = msg.angular.z

                # Set robot to walking mode
                self.cmd2go.mode = 2   # 2 corresponds to walking mode

            if self.currentKey == 'w' or self.currentKey == 'W':
                self.cmd2go.mode = 2 # Walk mode
                x_vel = 0.4

            elif self.currentKey == 's' or self.currentKey == 'S':
                self.cmd2go.mode = 2 # Stay in Walk mode, but stop moving
                x_vel = 0.0
                yaw_vel = 0.0

            elif self.currentKey == 'x' or self.currentKey == 'X':
                self.cmd2go.mode = 2 # Walk mode
                x_vel = -0.4

            elif self.currentKey == 'g' or self.currentKey == 'G':
                self.cmd2go.mode = 7 # Damping mode
                x_vel = 0.0

            elif self.currentKey == 'h' or self.currentKey == 'H':
                self.cmd2go.mode = 6  # Stand Up 

            elif self.currentKey == 'k' or self.currentKey == 'K': 
                self.cmd2go.mode = 2    # Turning Mode
                x_vel = 0.0
                yaw_vel = 0.3
            
            else:
                self.makeRobotStand()
            
            self.cmd2go.gaitType = 1
            self.cmd2go.velocity = [x_vel, 0]
            self.cmd2go.yawSpeed = yaw_vel
            self.udp.SetSend(self.cmd2go)
            self.udp.Send()
            
        else:
            self.makeRobotStand()
    
    def makeRobotStand(self):

        self.cmd2go.mode = 6
        self.cmd2go.velocity = [0, 0]
        self.cmd2go.yawSpeed = 0

        self.udp.SetSend(self.cmd2go)
        self.udp.Send()

        self.isStanding = True
            

def main(args=None):
    rclpy.init(args=args)
    executor = CommandExecutor()
    rclpy.spin(executor)
    executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        

        
        
    

