import rclpy
import copy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseStamped

class MyCar(Node):
    def __init__(self):
        super().__init__('mycar_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_waypoint = self.create_publisher(String, '/waypoint', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.timer2 = self.create_timer(0.1, self.timer_callback2)
        self.subscriber = self.create_subscription(Float32, '/pinky_battery_present', self.battery_callback, 10)
        self.create_subscription(PoseStamped, '/tracked_pose', self.pose_callback, 10)

        self.msg = Twist()
        self.temp_msg = None
        self.battery = 0
        self.waypoint = String()

    def timer_callback(self):
        if self.temp_msg != self.msg:
            self.temp_msg = copy.deepcopy(self.msg)
            self.publisher.publish(self.msg)

    def timer_callback2(self):
        self.pub_waypoint.publish(self.waypoint)

    def battery_callback(self, msg):
        self.battery = msg.data

    def pose_callback(self, msg):
        self.pos = msg

def main(args=None):
    rclpy.init(args=args)
    pinky_publisher = MyCar()
    rclpy.spin(pinky_publisher)
    pinky_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
