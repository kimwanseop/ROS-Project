import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MyCar(Node):
    def __init__(self):
        super().__init__('mycar_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.msg = Twist()

    def timer_callback(self):
        self.publisher.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    pinky_publisher = MyCar()
    rclpy.spin(pinky_publisher)
    pinky_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
