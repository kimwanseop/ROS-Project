import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from central_msg.srv import Choosepath, Obstacle, Redlight
from std_msgs.msg import String
from ultralytics import YOLO
from std_msgs.msg import Float32MultiArray
from scipy.special import binom
from matplotlib.patches import Polygon
import time
import socket
import cv2
import numpy as np
import torch
from central_msg.msg import Slope
import struct
import math
import gc
# import matplotlib
# matplotlib.use('Qt5Agg')  # 또는 'Qt5Agg'


class UdpReceiverNode(Node):
    def __init__(self, seg_model, det_model):
        super().__init__('udp_receiver_node')
        self.seg_model = seg_model
        self.det_model = det_model
        self.prev_redlight_status = False
        self.prev_obstacle_status = False

        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(("192.168.100.8", 8080))
        self.get_logger().info("Socket bound to 192.168.100.8:8080")

        self.sign_subscriber = self.create_subscription(String, "/sign", self.sign_callback, 10)
        self.publisher = self.create_publisher(Slope, "/slope", 10)
        self.signal_publisher = self.create_publisher(String, "/signal", 10)
        
        # self.avoid_timer = self.create_timer(0.033, self.avoid)
        self.execute_timer = self.create_timer(0.033, self.receive_and_command)

        self.srv_choosepath = self.create_service(Choosepath, "/pinky1/choosepath", self.choosepath_callback)
        self.srv_obstacle = self.create_service(Obstacle, "/pinky1/obstacle", self.obstacle_callback)
        self.srv_redlight = self.create_service(Redlight, "/pinky1/redlight", self.red_callback)
        
        self.__x, self.__y, self.__w, self.__h = 0, 400, 640, 240

        self.prev_mask = None
        self.unit = Conversion(1920, 1080, 16.1)
        self.center = Centroid()
        self.lookahead_distance = 60
        self.frame = None

        self.choosepath = False
        self.obstacle = False   
        self.redlight = False

        self.go_left = False
        self.child_protect = False
        self.prev_child_protect = False

        self.car_position = np.array([320, 940])
        self.prev_slope = 0.0
        self.slow_cnt = 0

    def choosepath_callback(self, request, response):
        self.get_logger().info(f"path: {request.is_left}")
        response.go_left = self.choosepath
        return response

    def obstacle_callback(self,request, response):
        self.get_logger().info(f"obstacle: {request.is_obstacle}")
        response.avoid = self.obstacle
        return response

    def red_callback(self, request, response):
        self.get_logger().info(f"red: {request.is_red}")
        response.stop = self.redlight
        return response

    def sign_callback(self, msg):
        if msg.data == "left":
            self.go_left = True
        if msg.data == "right":
            self.go_left = False

    def receive_and_command(self):
        try:
            data, addr = self.udp_socket.recvfrom(65536)
            # self.get_logger().info(f"Received data from {addr}")
            np_array = np.frombuffer(data, dtype=np.uint8)

            self.frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

            self.frame = cv2.rotate(self.frame, cv2.ROTATE_180)
            self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

            if self.frame is not None:
                frame_resized = cv2.resize(self.frame, (640, 640))

                self.ROI = frame_resized[self.__y:self.__y+self.__h, self.__x:self.__x+self.__w]

                seg_results = self.seg_model.predict(frame_resized)
                det_results = self.det_model.predict(frame_resized)
                
                # self.get_logger().info(det_results)
                """ object detection process """
                
                if det_results:
                    if self.redlight == False and (self.obstacle == True or self.child_protect == True):
                        self.slow_cnt += 1

                    if self.slow_cnt >= 200 and (self.obstacle == False or self.child_protect == False):
                        self.avoid = False
                        self.slow_cnt = 0
                        sign = String()
                        sign.data = "green light"
                        self.signal_publisher.publish(sign)

                    for det_result in det_results:
                    
                        classes = det_result.boxes.cls.cpu().numpy()
                        boxes = det_result.boxes.xyxy.cpu().numpy()  # (x_min, y_min, x_max, y_max)
                        
                        for cls_id, box in zip(classes, boxes):
                            for cls_id in classes:
                                object = self.det_model.names[int(cls_id)]
                                # self.get_logger().info(f"object: {object}, box: {box}")
                                sign = String()
                                sign.data = object

                                if object == "red light" and 250 < box[3] < 300 and abs(box[3]-box[1]) > 150:
                                    # self.get_logger().info(f"red light size: ((b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                                    self.redlight = True
                                    if self.prev_redlight_status == False:
                                        self.signal_publisher.publish(sign)
                                    self.prev_redlight_status = True

                                if object == "crossing" and box[1] > 550:
                                    # self.get_logger().info(f"crossing size: ((b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                                    if self.redlight != True:
                                        if self.prev_obstacle_status == False:
                                            # self.obstacle = True
                                            self.signal_publisher.publish(sign)
                                            # self.prev_obstacle_status = True

                                elif object in ["human", "obstacle", "goat"] and (420 < box[3] < 640) and (abs(box[3] - box[1]) > 150) and (250. < np.mean([box[2],box[0]]) < 390.):
                                    self.get_logger().info(f"self.object: {object}, {box}")
                                    if self.redlight != True:
                                        self.obstacle = True
                                        if self.prev_obstacle_status == False:
                                            if object == "human":
                                                self.get_logger().info(f"human size: (b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                                                
                                            if object == "obstacle":
                                                self.get_logger().info(f"car size: ((b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                                                
                                            if object == "goat":
                                                self.get_logger().info(f"goat size: ((b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                                        
                                            self.signal_publisher.publish(sign)
                                            self.prev_obstacle_status = True
                                    else:
                                        return
                                            
                                elif object == "child protect" and 250 < box[3] < 300 and abs(box[3]-box[1]) > 60:
                                    # self.get_logger().info(f"child protect size: ((b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                                    if self.redlight != True:
                                        self.signal_publisher.publish(sign)
                                        if self.prev_child_protect == False:
                                            self.child_protect = True
                                            self.prev_child_protect = True

                                elif object == "100KM":
                                    # self.get_logger().info(f"100KM size: ((b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                                    if self.redlight != True and self.child_protect:
                                        self.prev_child_protect = False
                                        self.child_protect = False
                                        self.signal_publisher.publish(sign)

                                elif object == "green light":
                                    self.redlight = False
                                    self.get_logger().info(f"green light size: ({box}, self.redlight: {self.redlight}, self.obstacle: {self.obstacle}")
                                    if self.obstacle != True:
                                        self.signal_publisher.publish(sign)
                                        self.prev_redlight_status = False
                else:
                    self.choosepath = False
                    self.obstacle = False
                    sign = String()
                    sign = "fine"
                    self.get_logger().info(f"lets go, I'm {sign}!")
                    self.signal_publisher.publish(sign)
                                

                """ segmentation process """
                for seg_result in seg_results:
                    
                    classes = seg_result.boxes.cls.cpu().numpy() #cls_id = {0: center, 1: right, 2: left, 7: safety zone}
                    masks = seg_result.masks.xy if seg_result.masks else None

                    if masks != None:
                        self.prev_masks = masks
                        for cls_id, mask in zip(classes, masks):
                            if self.obstacle and cls_id ==2:
                                self.get_logger().info(f"avoiding obstacle... {self.seg_model.names[int(cls_id)]}")
                                self.avoid = True
                                self.destination = mask

                            elif cls_id == 0:
                                self.destination = mask
                                
                    elif self.prev_masks:
                        for cls_id, mask in zip(classes, self.prev_masks):
                            if self.obstacle and cls_id ==2:
                                self.get_logger().info(f"avoiding obstacle... {self.seg_model.names[int(cls_id)]}")
                                self.avoid = True
                                self.destination = mask

                            elif cls_id == 0:
                                self.destination = mask

                    self.center.get_centroid(self.destination)

            processor = PurePursuit(self.destination, self.lookahead_distance)
            
            lookahead_distance, self.bezier_points = processor.get_bezier_points(self.car_position, (self.center.centroid_x, self.center.centroid_y))

            bezier_path = processor.bezier_curve(self.bezier_points)
            lookahead_point = processor.find_lookahead_point(bezier_path, self.car_position, lookahead_distance)
            # print(f"lookahead_point: {lookahead_point}")
            
            slope = self.get_slope(lookahead_point)
            try:
                if math.copysign(1,slope) != math.copysign(1,self.prev_slope) and np.abs(slope - self.prev_slope) > 10:
                    # print("<이상값 감지>")
                    # print(f"보정 전 slope: {slope:.3f} deg")
                    # print(f"보정 후 slope: {self.prev_slope:.3f} deg")
                    print(f"*****({slope})*****")
                    slope = self.prev_slope
                    # print("-------------")
                else:
                    print(f"slope: {slope:.3f} deg")
                    self.prev_slope = slope
            except:
                print("Prev_slope doesnt exist.")
            
            msg = Slope()

            msg.curr_slope = slope
            msg.target_slope = 0.0
            msg.diff = 0.0 - slope

            # print(msg.curr_slope, msg.target_slope, msg.diff)
            # self.get_logger().info(f"Publishing: {msg.curr_slope}, {msg.target_slope}, {msg.diff}")

            self.publisher.publish(msg)

            
        except Exception as e:
            self.get_logger().error(f"Error receiving or displaying image: {e}")
        
    def get_slope(self, lookahead_point):
        x1, y1 = self.car_position
        x2, y2 = lookahead_point

        delta_y = y1 - y2
        delta_x = x1 - x2

        desired_angle = math.atan2(delta_x, delta_y)

        vehicle_angle = 0

        steering_angle = desired_angle - vehicle_angle

        steering_angle = math.degrees((steering_angle + math.pi) % (2 * math.pi) - math.pi)

        return steering_angle

    
    def destroy_node(self):
        self.udp_socket.close()
        cv2.destroyAllWindows()
        self.get_logger().info("UDP Receiver Node stopped")
        super().destroy_node()

class PurePursuit():
    def __init__(self, lane_polygon, lookahead_distance):
        self.lane_polygon = lane_polygon
        self.lookahead_distance = lookahead_distance

    def get_bezier_points(self, car_position, centroid):
        
        trans_polygon = self.lane_polygon.copy()
        dest = self.find_nearest_value(trans_polygon[:, 0], centroid[0])
        target_y = trans_polygon[trans_polygon[:, 0]==dest][0][1]

        dist = centroid[1] - target_y

        if dist > 0:
            trans_polygon[:, 1] += int(dist)

            
        else:
            trans_polygon[:, 1] -= int(dist)
        
        
        if 0 <= np.abs(dist) <= 100:
            """ dist가 100이면 lah_d+=50
                dist가 50이면 lah_d+=100
                즉, 무게중심과 edge간 거리가 가까워질수록 lah_d는 비례증가"""
            self.lookahead_distance += (100 - np.abs(dist))
        

        sort_index = np.argsort(trans_polygon[:, 1])
        y_max = trans_polygon[sort_index[0]]

        if dist < 100:
            """ car_position ~ centroid"""
            mid_control1 = (car_position[0]-(car_position[0] - centroid[0]) / 3, 1000 - ((car_position[1] - centroid[1]) * 5 / 10))
            mid_control2 = (car_position[0]-(car_position[0] - centroid[0]) * 2 / 3, 1000- ((car_position[1] - centroid[1]) * 8 / 10))

            """ centroid ~ y_max"""
            mid_control3 = (centroid[0]-(centroid[0] - y_max[0]) / 3, centroid[1] - ((centroid[1] - y_max[1]) * 5 / 10))
            mid_control4 = (centroid[0]-(centroid[0] - y_max[0]) * 2 / 3, centroid[1]- ((centroid[1] - y_max[1]) * 8 / 10))
            
            return (self.lookahead_distance, (car_position, mid_control1, mid_control2, mid_control3, mid_control4, y_max))
        else:
            """ car_position ~ centroid"""
            mid_control1 = (car_position[0]-(car_position[0] - centroid[0]) / 3, 1000 - ((car_position[1] - centroid[1]) * 5 / 10))
            mid_control2 = (car_position[0]-(car_position[0] - centroid[0]) * 2 / 3, 1000- ((car_position[1] - centroid[1]) * 8 / 10))
            return (self.lookahead_distance, (car_position, mid_control1, mid_control2, centroid)) 
            

    def bezier_curve(self, bezier_points, num_points=100):
        n = len(bezier_points) - 1
        t_values = np.linspace(0, 1, num_points)
        curve = np.zeros((num_points, 2))
    
        for i in range(n + 1):
            bernstein_poly = binom(n, i) * (t_values ** i) * ((1 - t_values) ** (n - i))
            curve += np.outer(bernstein_poly, bezier_points[i])
        
        return curve

    def find_lookahead_point(self, curve, current_pos, lookahead_distance):
        distances = np.linalg.norm(curve - current_pos, axis=1)
        idx = np.argmin(np.abs(distances - lookahead_distance))
        return curve[idx]

    def find_nearest_value(self, arr, value):
        idx = np.argmin(np.abs(arr - value))
        return arr[idx]

class Conversion:
    def __init__(self, w_res, h_res, inch):
        self.__w_res = w_res
        self.__h_res = h_res
        self.__inch = inch

        self.__PPI = np.sqrt(np.power(self.__w_res, 2)+np.power(self.__h_res, 2))/self.__inch
        
        self.x = 0
        self.y = 0

    def p2cm(self):
        return  2.54 / self.__PPI

class Centroid():
    def __init__(self):
        self.centroid_x, self.centroid_y = 0, 0

    def get_centroid(self, polygon):
        area = 0
        self.centroid_x = 0
        self.centroid_y = 0
        n = len(polygon)

        for i in range(n):
            j = (i + 1) % n
            factor = polygon[i][0] * polygon[j][1] - polygon[j][0] * polygon[i][1]
            area += factor
            self.centroid_x += (polygon[i][0] + polygon[j][0]) * factor
            self.centroid_y += (polygon[i][1] + polygon[j][1]) * factor
        area /= 2.0
        if area != 0:
            self.centroid_x /= (6 * area)
            self.centroid_y /= (6 * area)



def main(args=None):
    seg_checkpoint_path = '/root/asap/data/best.pt'
    det_checkpoint_path = '/root/asap/data/det_best2.pt'

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Using device: {device}")

    seg_model = YOLO(seg_checkpoint_path, verbose=False).to(device)
    det_model = YOLO(det_checkpoint_path, verbose=False).to(device)

    rclpy.init(args=args)
    node = UdpReceiverNode(seg_model, det_model)
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()