import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist


class PublishGo1Twist(Node):
    def __init__(self):
        super().__init__('go1_twist_publisher')
        self.twistPub = self.create_publisher(Twist, 'go1_twist', 10)
        self.timer = self.create_timer(0.1, self.twistPubCallback)
    
    def twistPubCallback(self):

        msg = Twist()

        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.twistPub.publish(msg)
        self.get_logger().info(f'Published Twist: {[msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]}')
    
def main(args=None):
    rclpy.init(args=args)
    publisher = PublishGo1Twist()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        