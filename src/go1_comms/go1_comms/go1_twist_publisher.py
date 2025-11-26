import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Image
import math
import datetime
import time
import csv
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError

class PublishGo1Twist(Node):
    def __init__(self):
        super().__init__('go1_twist_publisher')

        # --- Publishers ---
        self.twistPub = self.create_publisher(Twist, 'go1_twist', 10)
        
        # --- Subscribers ---
        self.IMU_subscriber = self.create_subscription(
            Imu, 'go1_imu', self.imu_callback, 10)
            
        # Camera Subscribers
        self.bridge = CvBridge()
        self.latest_color_img = None
        self.latest_depth_img = None
        
        self.color_sub = self.create_subscription(
            Image, 'ext_cam1_color', self.color_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, 'ext_cam1_depth', self.depth_callback, 10)

        self.get_logger().info('Go1 Node Started. Waiting for Camera & IMU...')

        # --- File System Setup ---
        # Create a folder in the current execution directory
        self.base_path = os.path.join(os.getcwd(), "go1_captured_data")
        self.color_path = os.path.join(self.base_path, "color")
        self.depth_path = os.path.join(self.base_path, "depth")
        
        os.makedirs(self.color_path, exist_ok=True)
        os.makedirs(self.depth_path, exist_ok=True)
        self.get_logger().info(f"Saving images to: {self.base_path}")

        # --- Control Parameters ---
        self.forward_speed = 0.20
        self.total_forward_time = 8.0   # Total time for one side
        self.half_time = self.total_forward_time / 2.0
        
        self.pre_turn_delay = 1.0
        self.post_turn_delay = 1.0
        self.capture_delay = 2.0        # Time to stop at midpoint to ensure clean image
        
        self.yaw_tolerance = 0.03
        self.turn_speed = 0.4 
        self.kp_straight = 0.5
        self.turn_overshoot_deg = 2.0   

        # --- State Variables ---
        self.imu_data = None
        self.yaw = 0.0
        
        # States: INIT_ALIGN -> FORWARD_1 -> CAPTURE -> FORWARD_2 -> PRE_TURN ...
        self.state = "INIT_ALIGN" 
        self.sides_done = 0
        self.state_start_time = self.get_clock().now()
        
        self.fixed_heading_for_leg = 0.0 
        self.turn_start_yaw = 0.0
        self.target_yaw = 0.0

        # Logging
        timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = f"imu_log_{timestamp_str}.csv"
        self.log_file = open(self.log_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(['timestamp', 'yaw', 'target_yaw', 'state'])
        
        self.timer = self.create_timer(0.1, self.control_logic) 

    # --- Callbacks ---
    def imu_callback(self, msg):
        self.imu_data = msg
        self.yaw = msg.orientation.z % (2 * math.pi)
        
        current_time = time.time()
        self.csv_writer.writerow([current_time, self.yaw, self.target_yaw, self.state])
        
        if self.state == "INIT_ALIGN":
            self.fixed_heading_for_leg = self.yaw 
            self.state_start_time = self.get_clock().now()
            self.state = "FORWARD_1"
            self.get_logger().info('Aligned. Starting Leg 1 (First Half).')

    def color_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV Image (BGR)
            self.latest_color_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")

    def depth_callback(self, msg):
        try:
            # Pass through (usually 16UC1 or 32FC1). passthrough keeps original encoding
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")

    # --- Helper: Save Image ---
    def save_snapshot(self):
        timestamp = datetime.datetime.now().strftime("%H%M%S")
        filename_base = f"leg_{self.sides_done + 1}_midpoint_{timestamp}"
        
        # Save Color
        if self.latest_color_img is not None:
            c_name = os.path.join(self.color_path, f"{filename_base}.jpg")
            cv2.imwrite(c_name, self.latest_color_img)
            self.get_logger().info(f"Saved Color: {c_name}")
        else:
            self.get_logger().warn("No Color Image available to save!")

        # Save Depth
        if self.latest_depth_img is not None:
            d_name = os.path.join(self.depth_path, f"{filename_base}.png")
            cv2.imwrite(d_name, self.latest_depth_img)
            self.get_logger().info(f"Saved Depth: {d_name}")
        else:
            self.get_logger().warn("No Depth Image available to save!")

    # --- State Handlers ---

    def handle_forward_generic(self, next_state_name):
        """ 
        Generic handler for moving forward. 
        Args: next_state_name (str) -> Where to go after time expires
        """
        twist = Twist()
        twist.linear.x = self.forward_speed
        
        # Heading correction
        heading_error = (self.fixed_heading_for_leg - self.yaw + math.pi) % (2 * math.pi) - math.pi
        twist.angular.z = self.kp_straight * heading_error

        elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
        
        # We only walk for HALF the total time here
        if elapsed >= self.half_time:
            # Stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.twistPub.publish(twist)
            
            self.state = next_state_name
            self.state_start_time = self.get_clock().now()
            self.get_logger().info(f'Segment Done. Switching to {next_state_name}')
        else:
            self.twistPub.publish(twist)

    def handle_capture(self):
        """ Stop at midpoint and save data """
        twist = Twist() # Send zeros
        self.twistPub.publish(twist)
        
        elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
        
        # Wait a bit to ensure robot is still and camera buffer is fresh
        if elapsed >= self.capture_delay:
            self.get_logger().info("Capturing Images...")
            self.save_snapshot()
            
            # Resume movement
            self.state = "FORWARD_2"
            self.state_start_time = self.get_clock().now()
            self.get_logger().info("Resuming Motion (Second Half).")

    def handle_pre_turn(self):
        twist = Twist()
        self.twistPub.publish(twist)
        elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
        
        if elapsed >= self.pre_turn_delay:
            self.turn_start_yaw = self.yaw
            
            # Overshoot Calculation
            overshoot_rad = math.radians(self.turn_overshoot_deg)
            total_turn_rad = (math.pi / 2) + overshoot_rad
            self.target_yaw = (self.turn_start_yaw - total_turn_rad) % (2 * math.pi)
            
            self.state = "TURN"
            self.get_logger().info(f'Turning...')

    def handle_turn(self):
        twist = Twist()
        yaw_error = (self.target_yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi

        if abs(yaw_error) > self.yaw_tolerance:
            if yaw_error > 0:
                twist.angular.z = self.turn_speed 
            else:
                twist.angular.z = -self.turn_speed 
            self.twistPub.publish(twist)
        else:
            twist.angular.z = 0.0
            self.twistPub.publish(twist)
            self.sides_done += 1
            
            if self.sides_done >= 4:
                self.state = "STOP"
                self.get_logger().info('Square complete. Stopping.')
            else:
                self.state = "POST_TURN"
                self.state_start_time = self.get_clock().now()

    def handle_post_turn(self):
        twist = Twist()
        self.twistPub.publish(twist)
        elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
        
        if elapsed >= self.post_turn_delay:
            self.fixed_heading_for_leg = self.target_yaw 
            
            # Start the FIRST half of the next leg
            self.state = "FORWARD_1"
            self.state_start_time = self.get_clock().now()
            self.get_logger().info('Stabilized. Starting Leg Part 1.')

    def handle_stop(self):
        twist = Twist() 
        self.twistPub.publish(twist)
    
    def control_logic(self):
        if self.imu_data is None and self.state == "INIT_ALIGN": return
        
        if self.state == 'FORWARD_1':
            # Go Forward 1st half -> Then Capture
            self.handle_forward_generic("CAPTURE")
            
        elif self.state == 'CAPTURE':
            self.handle_capture()
            
        elif self.state == 'FORWARD_2':
            # Go Forward 2nd half -> Then Pre-Turn
            self.handle_forward_generic("PRE_TURN")
            
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