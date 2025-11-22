import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math
import datetime
import time
import csv

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

        # --- Control Parameters ---
        self.forward_speed = 0.20
        self.forward_time = 8.0
        
        self.pre_turn_delay = 1.0
        self.post_turn_delay = 1.0
        
        self.yaw_tolerance = 0.03       # approx 1.7 degrees
        self.turn_speed = 0.4 
        self.kp_straight = 0.5

        # --- DRIFT CORRECTION (YOUR STRATEGY) ---
        # If the robot drifts outwards, we turn MORE than 90 degrees to aim back in.
        # Positive value = turn MORE (e.g., 2.0 means turn 92 degrees total)
        # Negative value = turn LESS (e.g., -2.0 means turn 88 degrees total)
        self.turn_overshoot_deg = 2.0   
        
        # --- State Variables ---
        self.imu_data = None
        self.yaw = 0.0
        self.state = "INIT_ALIGN"
        self.sides_done = 0
        self.state_start_time = self.get_clock().now()
        self.fixed_heading_for_leg = 0.0 
        self.turn_start_yaw = 0.0
        self.target_yaw = 0.0
        
        self.get_logger().info('FSM controller init. Waiting for first IMU message...')

        timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = f"imu_log_{timestamp_str}.csv"
        self.log_file = open(self.log_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(['timestamp', 'yaw', 'target_yaw', 'state'])
        
        self.timer = self.create_timer(0.1, self.control_logic) 

    def imu_callback(self, msg):
        self.imu_data = msg
        self.yaw = msg.orientation.z % (2 * math.pi)
        
        current_time = time.time()
        self.csv_writer.writerow([current_time, self.yaw, self.target_yaw, self.state])
        
        if self.state == "INIT_ALIGN":
            self.fixed_heading_for_leg = self.yaw 
            self.state_start_time = self.get_clock().now()
            self.state = "FORWARD"
            self.get_logger().info('Aligned. Moving Forward.')

    #---------------------------------------------------------
    # State Handlers
    #---------------------------------------------------------
    
    def handle_forward(self):
        twist = Twist()
        twist.linear.x = self.forward_speed
        
        # Maintain the heading established at the start of this leg
        heading_error = (self.fixed_heading_for_leg - self.yaw + math.pi) % (2 * math.pi) - math.pi
        twist.angular.z = self.kp_straight * heading_error

        elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
        
        if elapsed >= self.forward_time:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.twistPub.publish(twist)
            self.state = "PRE_TURN"
            self.state_start_time = self.get_clock().now()
            self.get_logger().info('Forward Done. Pausing...')
        else:
            self.twistPub.publish(twist)

    def handle_pre_turn(self):
        twist = Twist()
        self.twistPub.publish(twist)

        elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
        
        if elapsed >= self.pre_turn_delay:
            self.turn_start_yaw = self.yaw
            
            # --- CALCULATE TARGET WITH OVERSHOOT ---
            # 1. Convert overshoot to radians
            overshoot_rad = math.radians(self.turn_overshoot_deg)
            
            # 2. Total turn angle = 90 degrees (pi/2) + Overshoot
            total_turn_rad = (math.pi / 2) + overshoot_rad
            
            # 3. Calculate Target (Right turn = subtract angle)
            self.target_yaw = (self.turn_start_yaw - total_turn_rad) % (2 * math.pi)
            
            self.state = "TURN"
            self.get_logger().info(f'Target set: {math.degrees(total_turn_rad):.2f} deg turn.')

    def handle_turn(self):
        twist = Twist()
        yaw_error = (self.target_yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi

        if abs(yaw_error) > self.yaw_tolerance:
            # No Linear compensation, just rotate
            if yaw_error > 0:
                twist.angular.z = self.turn_speed 
            else:
                twist.angular.z = -self.turn_speed 
            
            self.twistPub.publish(twist)
        else:
            twist.angular.z = 0.0
            self.twistPub.publish(twist)
            
            self.sides_done += 1
            self.get_logger().info(f'Turn Complete. Entering Post-Turn Wait.')

            if self.sides_done >= 4:
                self.state = "STOP"
            else:
                self.state = "POST_TURN"
                self.state_start_time = self.get_clock().now()

    def handle_post_turn(self):
        twist = Twist()
        self.twistPub.publish(twist)
        elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
        
        if elapsed >= self.post_turn_delay:
            # IMPORTANT:
            # Even though we turned 92 degrees, we lock THIS new heading as "Straight".
            # This effectively rotates the entire square frame by 2 degrees every corner.
            self.fixed_heading_for_leg = self.target_yaw 
            
            self.state = "FORWARD"
            self.state_start_time = self.get_clock().now()
            self.get_logger().info('Stabilized. Starting next leg.')

    def handle_stop(self):
        twist = Twist() 
        self.twistPub.publish(twist)
    
    def control_logic(self):
        if self.imu_data is None and self.state == "INIT_ALIGN": return
        if self.state == 'FORWARD': self.handle_forward()
        elif self.state == 'PRE_TURN': self.handle_pre_turn()
        elif self.state == 'TURN': self.handle_turn()
        elif self.state == 'POST_TURN': self.handle_post_turn()
        elif self.state == 'STOP': self.handle_stop()
        elif self.state == 'INIT_ALIGN': pass 
        else: self.state = 'STOP'

def main(args=None):
    rclpy.init(args=args)
    publisher = PublishGo1Twist()
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info('Shutting down...')
    finally:
        publisher.handle_stop()
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()