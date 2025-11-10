import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math


class PublishGo1Twist(Node):
    def __init__(self):
        super().__init__('go1_twist_publisher')


        self.twistPub = self.create_publisher(Twist, 'go1_twist', 10)
        self.get_logger().info('Go1 Twist Publisher Node has been started.')
        self.IMU_subscriber = self.create_subscription(
            Imu,
            'go1_imu',
            self.imu_callback,
            10)
 
        self.timer = self.create_timer(0.1, self.control_logic)

        self.imu_data = None
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.forward_speed = 0.25
        self.turn_speed = 0.3
        self.forward_time = 5.0
        self.yaw_tolerance = 0.05

        self.state = "INIT_ALIGN"
        self.sides_done = 0
        self.start_time = self.get_clock().now()
        self.start_yaw = 0.0
        self.target_yaw = 0.0
        self.yaw = 0.0
        self.get_logger().info('FSM controller init.')

    def imu_callback(self, msg):

        
        """
        Definations:
        msg.orientation.x : Roll
        msg.orientation.y : Pitch
        msg.orientation.z : Yaw
        msg.orientation.w : not used

        msg.angular_velocity.x : Gyro X
        msg.angular_velocity.y : Gyro Y
        msg.angular_velocity.z : Gyro Z

        msg.linear_acceleration.x : Accel X
        msg.linear_acceleration.y : Accel Y
        msg.linear_acceleration.z : Accel Z          
        """
        self.imu_data = msg
        self.roll  = msg.orientation.x
        self.pitch = msg.orientation.y
        self.yaw   = msg.orientation.z % (2 * math.pi)  

    #---------------------------------------------------------
    # state handlers
    #---------------------------------------------------------
    
    def handle_init_align(self):
        """ initial alignment to 0 rad yaw """
        
        twist = Twist()
        if abs(self.yaw) > self.yaw_tolerance:
            twist.angular.z = -self.turn_speed if self.yaw > 0 else self.turn_speed
        else:
            self.start_time = self.get_clock().now()
            self.state = "FORWARD"
            self.start_yaw = self.yaw
            self.get_logger().info('Aligned. Moving Forward.')
        self.twistPub.publish(twist)

    def handle_forward(self):
        """ move forward for a set time """
        
        twist = Twist()
        twist.linear.x = self.forward_speed

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed >= self.forward_time:
            self.state = "TURN"
            self.start_yaw = self.yaw
            self.target_yaw = (self.start_yaw + math.pi/2) % (2 * math.pi)
            self.start_time = self.get_clock().now()
            self.get_logger().info('Forward complete. Turning.')

        
        else:
            twist.linear.x = self.forward_speed

        self.twistPub.publish(twist)    


    def handle_turn(self):
        """ turn 90 degrees """
        
        twist = Twist()
        yaw_error = (self.target_yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi

        if abs(yaw_error) > self.yaw_tolerance:
            twist.angular.z = self.turn_speed if yaw_error > 0 else -self.turn_speed
        else:
            self.sides_done += 1
            if self.sides_done >= 4:
                self.state = "STOP"
                self.get_logger().info('Square complete. Stopping.')
            else:
                self.state = "FORWARD"
                self.start_time = self.get_clock().now()
                self.start_yaw = self.yaw
                self.get_logger().info(f'Turn complete. Moving Forward. Sides done: {self.sides_done}')
        
        self.twistPub.publish(twist)

    def handle_stop(self):
        """ stop all movement """
        
        twist = Twist()
        self.twistPub.publish(twist)
        self.get_logger().info('Robot stopped.')
    
    def control_logic(self):
        if self.state == 'INIT_ALIGN':
            self.handle_init_align()
        elif self.state == 'FORWARD':
            self.handle_forward()
        elif self.state == 'TURN':
            self.handle_turn()
        elif self.state == 'STOP':
            self.handle_stop()
        else:
            self.get_logger().warn(f'Unknown state: {self.state}')
            self.state = 'STOP'

    
def main(args=None):
    rclpy.init(args=args)
    publisher = PublishGo1Twist()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        