#---------------------------------------------------------------
#---------------------------------------------------------------

# WHAT DOES THIS Node DO ?
# 1. Makes UDP Connection to the GO1 robot
# 2. Gets IMU data from GO1 robot and publihses to 'go1_imu' topic
# 3. Subscribes to 'keyboard_cmd' topic and registers the command
# 4. Runs the robot based on command at 10 Hz
# 4. Subscribes to 'go1_twist' topic and commands the GO1 robot that twist as callback.

#--------------------------------------------------------------
#--------------------------------------------------------------


# ROS2 Imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String

# Other Imports
import sys
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

        # TRY CONNECTING TO ROBOT
        try:
            self.udp = sdk.UDP(HIGHLEVEL, LOCAL_PORT, ROBOT_IP, ROBOT_PORT)
            self.cmd2go = sdk.HighCmd()
            self.gostate = sdk.HighState()
            self.udp.InitCmdData(self.cmd2go)
            self.get_logger().info(f"Connected to GO1 : {self.udp}")

        except Exception as e:
            self.get_logger().info(f"Couldn't connect to robot : {e}")
        
        self.imuPub = self.create_publisher(Imu, 'go1_imu', 10)
        self.imuTimer = self.create_timer(0.1, self.imuPublisher)

        self.twistSub = self.create_subscription(Twist, 'go1_twist', self.try2moveRobot, 10)
        self.kcmdSub = self.create_subscription(String, 'keyboard_cmd', self.registerCommand, 10)
        
        self.currentKey = 'S'
        self.isStanding = False
        self.op_mode = 'teleop'

        self.runFromCommandTimer = self.create_timer(0.1, self.moveRobotFromCmd)

    def registerCommand(self, msg):        
        self.currentKey = msg.data
        self.op_mode = 'teleop'
        self.get_logger().info(f"Command Registered : {self.currentKey}")

        self.moveRobotFromCmd()
        

    def moveRobotFromCmd(self):

        fwd_speed = 0.25
        
        if self.op_mode != 'teleop':
            return

        self.udp.Recv()
        self.udp.GetRecv(self.gostate)

        if self.isStanding:
            x_vel = 0
            yaw_vel = 0
            self.cmd2go.mode = 2

            if self.currentKey == 'w' or self.currentKey == 'W':
                self.cmd2go.mode = 2 # Walk mode
                x_vel = fwd_speed

            elif self.currentKey == 's' or self.currentKey == 'S':
                self.cmd2go.mode = 2 # Stay in Walk mode, but stop moving
                x_vel = 0.0
                yaw_vel = 0.0

            elif self.currentKey == 'x' or self.currentKey == 'X':
                self.cmd2go.mode = 2 # Walk mode
                x_vel = -fwd_speed

            elif self.currentKey == 'g' or self.currentKey == 'G':
                self.cmd2go.mode = 7 # Damping mode
                x_vel = 0.0

            elif self.currentKey == 'h' or self.currentKey == 'H':
                self.cmd2go.mode = 6  # Stand Up 

            elif self.currentKey == 'k' or self.currentKey == 'K': 
                self.cmd2go.mode = 2    # Turning Mode
                x_vel = 0.0
                yaw_vel = 0.3
            
            elif self.currentKey == 'l' or self.currentKey == 'L': 
                self.cmd2go.mode = 2    # Turning Mode
                x_vel = 0.0
                yaw_vel = -0.3
            
            else:
                self.makeRobotStand()
            
            self.cmd2go.gaitType = 1
            self.cmd2go.velocity = [x_vel, 0]
            self.cmd2go.yawSpeed = yaw_vel
            self.udp.SetSend(self.cmd2go)
            self.udp.Send()
            self.get_logger().info(f"Moving robot with x_vel : {x_vel}, yaw_vel: {yaw_vel}")
        
        else:
            self.makeRobotStand()


    def try2moveRobot(self, msg):

        self.op_mode = 'from_twist'

        self.udp.Recv()
        self.udp.GetRecv(self.gostate)

        x_vel = msg.linear.x
        yaw_vel = msg.angular.z

        self.cmd2go.gaitType = 1
        self.cmd2go.velocity = [x_vel, 0]
        self.cmd2go.yawSpeed = yaw_vel
        self.udp.SetSend(self.cmd2go)
        response = self.udp.Send()

        self.get_logger().info(f"Moving robot with x_vel : {x_vel}, yaw_vel: {yaw_vel}, response : {response}")

    
    def makeRobotStand(self):

        self.udp.Recv()
        self.udp.GetRecv(self.gostate)

        self.cmd2go.mode = 6
        self.cmd2go.velocity = [0, 0]
        self.cmd2go.yawSpeed = 0

        self.udp.SetSend(self.cmd2go)
        self.udp.Send()

        self.isStanding = True
    
    def imuPublisher(self):

        self.udp.Recv()
        self.udp.GetRecv(self.gostate)

        msg = Imu()
        
        acc = self.gostate.imu.accelerometer
        gyro = self.gostate.imu.gyroscope
        rpy = self.gostate.imu.rpy

        msg.linear_acceleration.x = acc[0]
        msg.linear_acceleration.y = acc[1]
        msg.linear_acceleration.z = acc[2]

        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.x = gyro[1]
        msg.angular_velocity.x = gyro[2]

        # Using quaternion msg definition to send rpy(roll, pitch, yaw)
        msg.orientation.x = rpy[0]
        msg.orientation.y = rpy[1]
        msg.orientation.z = rpy[2]
        msg.orientation.w = 1.0          # Not useful

        self.imuPub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    executor = CommandExecutor()
    rclpy.spin(executor)
    executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        

        
        
    

