import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from central_msg.msg import Slope
from central_msg.srv import Choosepath, Obstacle, Redlight
from std_msgs.msg import String
import random

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        
        self.cross_road_cnt = 0
        self.absolute_linear_x = 0.25

        self.__fix__ = 0.25
        self._fix_linear_x = self.__fix__
        self.linear_Kp = 0.1

        #횡방향 게인
        self.declare_parameter('Kp', 1.8)
        self.declare_parameter('Ki', 0.11)
        self.declare_parameter('Kd', 0.08)

        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value

        self.prev_output = 0.0
        self.integral_limit = 10.0
        self.integral_lower_limit = -10.0
        

        #종방향 게인
        # self.declare_parameter('l_Kp', 1.0)
        # self.declare_parameter('l_Ki', 0.07)
        # self.declare_parameter('l_Kd', 0.6)

        # self.l_Kp = self.get_parameter('l_Kp').value
        # self.l_Ki = self.get_parameter('l_Ki').value
        # self.l_Kd = self.get_parameter('l_Kd').value

        # self.linear_integral_limit = self.__fix__ * 0.2
        # self.linear_integral_lower_limit = -self.linear_integral_limit

        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sign_publisher = self.create_publisher(String, '/sign', 10)
        
        self.create_subscription(Slope, '/slope', self.slope_callback, 10)
        self.create_subscription(String, '/signal', self.detection_callback, 10)

        self.create_timer(0.03, self.linear_controller)

        self.client_choosepath = self.create_client(Choosepath, "/pinky1/choosepath")
        self.client_obstacle = self.create_client(Obstacle, "/pinky1/obstacle")
        self.client_redlight = self.create_client(Redlight, "/pinky1/redlight")

        self.prev_linear_x_error = 0.0
        self.linear_x_integral = 0.0
        self.target_linear_x = self.__fix__

        self.prev_error = 0.0
        self.integral = 0.0
        self.current_slope = 0.0
        
        self.target_slope = 0.0
        self.error = 0.0



    def send_choosepath_request(self, is_left):
        req = Choosepath.Request()
        req.is_left = is_left
        future = self.client_choosepath.call_async(req)
        return future

    def send_obstacle_request(self, is_obstacle):
        req = Obstacle.Request()
        req.is_obstacle = is_obstacle
        future = self.client_obstacle.call_async(req)
        return future

    def send_redlight_request(self, is_red):
        req = Redlight.Request()
        req.is_red = is_red
        future = self.client_redlight.call_async(req)
        return future


    def Go(self):
        if self._fix_linear_x >= self.__fix__:
            self._fix_linear_x = self.__fix__
            return
        else:    
            self._fix_linear_x += 0.0001

    def Stop(self):
        if self._fix_linear_x <= 0.0:
            self._fix_linear_x = 0
            return
        else:
            self._fix_linear_x -= 0.0001


    def detection_callback(self, msg): # 종방향 속도 제어
        self.get_logger().info(f'msg.data: {msg.data}')
        match msg.data:
            case "green light":
                self.target_linear_x = 0.25
            case "100KM":
                self.target_linear_x = 0.25
            case "crossing":
                self.target_linear_x = 0.20
            case "child protect":
                self.target_linear_x = 0.15
            case ("goat" | "human" | "obstacle"):
                self.target_linear_x = 0.14
            case "red light":
                self.target_linear_x = 0.0


        # match msg.data:
        #     case "go":
        #         pass
        #     case "no right":
        #         pass
        #     case "right":
        #         pass
        #     case "left":
        #         pass
        
        # match msg.data:
        #     case "pinky":
        #         pass
        #     case "cross road":
        #         pass


    def linear_controller(self):
        error = self.target_linear_x - self._fix_linear_x

        # self.get_logger().info(f'curr_speed: {self._fix_linear_x}, target_speed: {self.target_linear_x}')
            
        if self.target_linear_x > self._fix_linear_x:
            self._fix_linear_x += self.linear_Kp * error
        else:
            self._fix_linear_x -= self.linear_Kp * abs(error)

        if 0.0 <= self._fix_linear_x <= 0.0001:
            self._fix_linear_x = 0.0
            
        if self._fix_linear_x > self.__fix__:
            self._fix_linear_x = self.__fix__

            
    # 횡방향 속도 제어
    def slope_callback(self, msg):
        if self.target_linear_x <= 0.17:
            self.Kp = 1.2
        else:
            self.Kp = 1.8

        error = msg.target_slope - msg.curr_slope
        self.integral += error
        derivative = error - self.prev_error
        
        if abs(error) < 0.01: 
            self.integral = 0.0 

        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        elif self.integral < self.integral_lower_limit:
            self.integral = self.integral_lower_limit

        pid_output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        
        pid_output /= -45.0

        if pid_output >= 0.99:
            pid_output = 0.99
        elif pid_output <= -0.99:
            pid_output = -0.99

        # self.get_logger().info(f'Target Slope: {msg.target_slope}, Current Slope: {msg.curr_slope}, Error: {error}, PID Output: {pid_output}')
        
        if self._fix_linear_x <= 0.02:
            pid_output = 0.0
            
        cmd_vel = Twist()
        cmd_vel.linear.x = self._fix_linear_x

        if cmd_vel.linear.x <= 0.02:
            pid_output = 0.0
        else:
            cmd_vel.angular.z = pid_output

        
        
        self.vel_publisher.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
