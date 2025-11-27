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
from go1_interfaces.msg import GoState
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import datetime, csv

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
        
        self.statePub = self.create_publisher(GoState, 'GoState', 10)
        self.stateTimer = self.create_timer(0.1, self.statePublisher)

        self.twistSub = self.create_subscription(Twist, 'go1_twist', self.try2moveRobot, 10)
        self.kcmdSub = self.create_subscription(String, 'keyboard_cmd', self.registerCommand, 10)
        
        self.currentKey = 'S'
        self.isStanding = False
        self.op_mode = 'teleop'

        self.runFromCommandTimer = self.create_timer(0.1, self.moveRobotFromCmd)

        # timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        # self.log_filename = f"imu_log_{timestamp_str}.csv"
        # self.log_file = open(self.log_filename, 'w', newline='')
        # self.csv_writer = csv.writer(self.log_file)
        # self.csv_writer.writerow(['timestamp', 'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z', 'r', 'p', 'y'])

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
    
    def statePublisher(self):

        self.udp.Recv()
        self.udp.GetRecv(self.gostate)

        msg = GoState()
        
        acc = self.gostate.imu.accelerometer
        gyro = self.gostate.imu.gyroscope
        rpy = self.gostate.imu.rpy

        go_x = self.gostate.position[0]
        go_y = self.gostate.position[1]
        # go_z = self.gostate.position[2]

        msg.go1_imu.linear_acceleration.x = acc[0]
        msg.go1_imu.linear_acceleration.y = acc[1]
        msg.go1_imu.linear_acceleration.z = acc[2]

        msg.go1_imu.angular_velocity.x = gyro[0]
        msg.go1_imu.angular_velocity.x = gyro[1]
        msg.go1_imu.angular_velocity.x = gyro[2]

        # Using quaternion msg definition to send rpy(roll, pitch, yaw)
        msg.go1_imu.orientation.x = rpy[0]
        msg.go1_imu.orientation.y = rpy[1]
        msg.go1_imu.orientation.z = rpy[2]
        msg.go1_imu.orientation.w = 1.0          # Not useful

        msg.go1_pose2d.x = go_x
        msg.go1_pose2d.y = go_y
        msg.go1_pose2d.theta = rpy[2]
        
        
        # current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        # self.csv_writer.writerow([current_time, acc[0], acc[1], acc[2],
        #                           gyro[0], gyro[1], gyro[2],
        #                           rpy[0], rpy[1], rpy[2]])
        

        self.statePub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    executor = CommandExecutor()
    rclpy.spin(executor)
    executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        

        
        
    

