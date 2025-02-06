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
        
        # 중앙선 검출 좌표 초기화
        self.central_x = 320
        self.central_y = 640

        self.prev_center = None
        self.unit = Conversion(1920, 1080, 16.1)
        self.center = Centroid()
        self.lookahead_distance = 60
        self.frame = None

        self.choosepath = False
        self.obstacle = False   
        self.redlight = False
        self.avoid = False
        self.go_left = False
        self.no_right = False
        self.child_protect = False
        self.prev_child_protect = False
        self.destination = None
        self.center_objective = None

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
            # assert data is not None, "Received data is None"

            # self.get_logger().info(f"Received data from {addr}")
            np_array = np.frombuffer(data, dtype=np.uint8)

            self.frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

            self.frame = cv2.rotate(self.frame, cv2.ROTATE_180)
            self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

            if self.frame is not None:
                frame_resized = cv2.resize(self.frame, (640, 640))

                self.ROI = frame_resized[self.__y:self.__y+self.__h, self.__x:self.__x+self.__w]

                try:
                    hsv = cv2.cvtColor(self.ROI, cv2.COLOR_BGR2HSV)

                    lower_yellow = np.array([20, 100, 100])
                    upper_yellow = np.array([30, 255, 255])
                    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

                    kernel = np.ones((5, 5), np.uint8)
                    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    if contours:
                        largest_contour = max(contours, key=cv2.contourArea)  # 가장 큰 윤곽선 선택
                        M = cv2.moments(largest_contour)
                        
                        if M["m00"] != 0:  # 무게중심이 존재하는 경우
                            cx_roi = int(M["m10"] / M["m00"])  # ROI 내부 X 중심 좌표
                            cy_roi = int(M["m01"] / M["m00"])  # ROI 내부 Y 중심 좌표

                            # ROI 내부 좌표를 원본 이미지 좌표로 변환
                            self.central_x = cx_roi + self.__x
                            self.central_y = cy_roi + self.__y
                    
                    # self.get_logger().info(f"self.central_x: {self.central_x:.2f}, self.central_y: {self.central_y:.2f}")
                except:
                    pass
                #     else:
                #         pass
                # else:
                #     print("ROI 내에 황색 선이 검출되지 않음")

                seg_results = self.seg_model.predict(frame_resized)
                det_results = self.det_model.predict(frame_resized)
                
                # self.get_logger().info(det_results)
                """ object detection process """
                
                if det_results:
                    if self.redlight == False and (self.obstacle == True or self.child_protect == True):
                        self.slow_cnt += 1


                    #  장애물 혹은 어린이보호구역이 아닌 상태로 200count가 지나면 재가속
                    if self.slow_cnt >= 200 and (self.obstacle == False and self.child_protect == False):
                        self.avoid = False
                        self.slow_cnt = 0
                        sign = String()
                        sign.data = "reaccelerate"
                        self.signal_publisher.publish(sign)

                    for det_result in det_results:
                
                        classes = det_result.boxes.cls.cpu().numpy()
                        boxes = det_result.boxes.xyxy.cpu().numpy()  # (x_min, y_min, x_max, y_max)
                        confidences = det_result.boxes.conf.cpu().numpy()

                        """ 우선 conf score 높은 장애물이 감지되면 obstacle 신호를 먼저 정하기.
                            1. 장애물 감지되면 obstacle = True
                            2. 장애물 안 보이면 obstacle = False
                                이건 이후 segmentation에서 장애물 회피동작에 영향을 줌."""
                        
                        # cnt = 0

                        # for cls_id, conf in zip(classes, confidences):
                        #     if self.det_model.names[int(cls_id)] in ["obstacle", "goat"] and conf > 0.7:
                        #         cnt += 1

                        # if cnt == 0:
                        #     self.get_logger().info("self.obstacle is False now")
                        #     self.obstacle = False
                            
                                
                        for cls_id, box, conf in zip(classes, boxes, confidences):
                            object_ = self.det_model.names[int(cls_id)]
                            self.get_logger().info(f"object_: {object_}, conf: {conf}, box: {box}")
                            sign = String()
                            sign.data = object_

                            if object_ == "red light" and 250 < box[3] < 300 and abs(box[3] - box[1]) > 150 and conf > 0.7:
                                # self.get_logger().info(f"red light size: ((b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                                self.redlight = True

                            if object_ == "cross road" :#and 270 < box[2] - box[0] < 370 and 500 < box[3] < 600  and conf > 0.7:
                                self.get_logger().info(f"cross road size: ((b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                                if self.redlight:
                                    if self.prev_redlight_status == False:
                                        self.signal_publisher.publish(sign)
                                    self.prev_redlight_status = True
                                else:
                                    """여기다가 내비게이션 interaction 코드 추가"""

                            if object_ == "crossing" and box[1] > 530 and conf > 0.8:
                                # self.get_logger().info(f"crossing size: ((b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                                if self.redlight == True:
                                    sign.data = "red light"
                                    self.signal_publisher.publish(sign)
                                if self.redlight != True:
                                    if self.prev_obstacle_status == False:
                                        # self.obstacle = True
                                        self.signal_publisher.publish(sign)
                                        # self.prev_obstacle_status = True

                            elif object_ in ["human", "goat"] and (380 < box[3] < 640) and (abs(box[3] - box[1]) > 200) and (290. < np.mean([box[2],box[0]]) < 370.) and conf > 0.8:
                                self.get_logger().info(f"self.object_: {object_}, {conf:.2f}, {box}, self.prev_obstacle_status: {self.prev_obstacle_status}")
                                if self.redlight != True:
                                    self.obstacle = True
                                    if self.prev_obstacle_status == False:
                                        # if object_ == "human":
                                            # self.get_logger().info(f"human size: (b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                                        # if object_ == "goat":
                                            # self.get_logger().info(f"goat size: ((b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                                    
                                        self.signal_publisher.publish(sign)
                                        self.prev_obstacle_status = True
                            
                            elif object_ == "obstacle" and (340 < box[3] < 640) and (abs(box[3] - box[1]) > 150) and (290. < np.mean([box[2],box[0]]) < 370.) and conf > 0.3:
                                self.get_logger().info(f"self.object_: {object_}, {conf:.2f}, {box}, self.prev_obstacle_status: {self.prev_obstacle_status}")
                                if self.redlight != True:
                                    self.obstacle = True
                                    if self.prev_obstacle_status == False:
                                        # self.get_logger().info(f"car size: ((b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                                        self.signal_publisher.publish(sign)
                                        self.prev_obstacle_status = True
                                            
                            elif object_ == "child protect" and 250 < box[3] < 300 and abs(box[3]-box[1]) > 60 and conf > 0.8:
                                # self.get_logger().info(f"child protect size: ((b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                                if self.redlight != True:
                                    self.signal_publisher.publish(sign)
                                    if self.prev_child_protect == False:
                                        self.child_protect = True
                                        self.prev_child_protect = True

                            elif object_ == "100KM" and conf > 0.8:
                                # self.get_logger().info(f"100KM size: ((b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                                if self.redlight != True and self.child_protect:
                                    self.prev_child_protect = False
                                    self.child_protect = False
                                    self.signal_publisher.publish(sign)

                            elif object_ == "green light" and conf > 0.8:
                                self.redlight = False
                                # self.get_logger().info(f"green light size: ({box}, self.redlight: {self.redlight}, self.obstacle: {self.obstacle}")
                                if self.obstacle != True:
                                    self.signal_publisher.publish(sign)
                                    self.prev_redlight_status = False
            
                            elif object_ == "no right" and 300 < np.mean([box[0], box[2]]) < 340 and conf > 0.8:
                                self.no_right = True
                                self.prev_no_right = True
                
                else:
                    self.choosepath = False
                    self.obstacle = False
                    sign = String()
                    sign.data = "fine"
                    self.get_logger().info(f"lets go, I'm {sign}!")
                    self.signal_publisher.publish(sign)
                                

                """ segmentation process """
                for seg_result in seg_results:
                    
                    classes = seg_result.boxes.cls.cpu().numpy() #cls_id = {0: center, 1: right, 2: left, 7: safety zone}
                    masks = seg_result.masks.xy if seg_result.masks else None
                    confidences = seg_result.boxes.conf.cpu().numpy()
                    
                    if self.no_right:
                        center_cnt = 0
                        for cls_id, mask, conf in zip(classes, masks, confidences):
                            if cls_id == 0:
                                center_cnt += 1

                        if center_cnt > 1:
                            center_cnt 


                    if masks is not None and len(masks) > 0:
                        # self.prev_masks = masks
                        for cls_id, mask, conf in zip(classes, masks, confidences):
                            center_list = []
                            # self.get_logger().info(f"seg_클래스 ID: {self.seg_model.names[int(cls_id)]}, seg_신뢰도: {conf:.2f}")
                            if self.obstacle and cls_id ==2:
                                self.get_logger().info(f"avoiding obstacle... to {self.seg_model.names[int(cls_id)]}")
                                self.center_objective = cls_id
                                self.avoid = True
                                self.destination = mask

                            elif self.obstacle and (cls_id == 0 or cls_id == 1) and self.central_x > 320:
                                self.get_logger().info(f"avoiding obstacle... to {self.seg_model.names[int(cls_id)]}")
                                self.center_objective = cls_id # 내가 목표로 했던 차로를 기억
                                self.avoid = True
                                self.destination = mask

                            elif self.obstacle == False and self.avoid and self.central_x > 320 and cls_id == 1 and conf > 0.85:
                                self.get_logger().info(f"return to line... {self.seg_model.names[int(cls_id)]}")

                                self.destination = mask

                            elif self.obstacle == False and self.avoid and self.central_x < 320 and cls_id == 2 and conf > 0.85:
                                self.get_logger().info(f"return to line... {self.seg_model.names[int(cls_id)]}")

                                self.destination = mask

                            # elif self.obstacle == False and cls_id == self.center_objective and conf > 0.85:
                            #     self.avoid = False
                            #     self.obstacle = False

                            elif cls_id == 0:
                                if self.center_objective:
                                    self.get_logger().info(f"I've got in {self.seg_model.names[int(cls_id)]}, seg_신뢰도: {conf:.2f}, 이전 차로: {self.seg_model.names[int(self.center_objective)]}")
                                    if self.center_objective == 1:
                                        self.next_dest = 2
                                    elif self.center_objective == 2:
                                        self.next_dest = 1

                                if self.no_right:
                                    imsi = Centroid()
                                    imsi.get_centroid(mask)
                                    center_list.append([imsi.centroid_x, mask])
                                else:
                                    self.destination = mask
                                    self.avoid = False

                        if len(center_list)>0:
                            values = [center[0] for center in center_list]
                            index = center_list[np.argmin(values)]
                            self.destination = center_list[index][1]

                            self.prev_center = self.destination
                            self.get_logger().info(self.destination)

                                
                    elif len(self.prev_center) > 0:
                    
                        self.destination = self.prev_center
                        # for cls_id, mask, conf in zip(classes, self.prev_masks, confidences):
                        #     # self.get_logger().info(f"seg_클래스 ID: {self.seg_model.names[int(cls_id)]}, seg_신뢰도: {conf:.2f}")
                        #     if self.obstacle and cls_id ==2 and self.central_x < 320:
                        #         self.get_logger().info(f"avoiding obstacle... to {self.seg_model.names[int(cls_id)]}")
                        #         self.center_objective = cls_id # 내가 목표로 했던 차로를 기억
                        #         self.avoid = True
                        #         self.destination = mask

                        #     elif self.obstacle and (cls_id == 0 or cls_id == 1) and self.central_x > 320:
                        #         self.get_logger().info(f"avoiding obstacle... to {self.seg_model.names[int(cls_id)]}")
                        #         self.center_objective = cls_id # 내가 목표로 했던 차로를 기억
                        #         self.avoid = True
                        #         self.destination = mask

                        #     elif self.obstacle == False and self.avoid and self.central_x > 320 and cls_id == 1 and conf > 0.85:
                        #         self.get_logger().info(f"return to line... {self.seg_model.names[int(cls_id)]}")

                        #         self.destination = mask

                        #     elif self.obstacle == False and self.avoid and self.central_x < 320 and cls_id == 2 and conf > 0.85:
                        #         self.get_logger().info(f"return to line... {self.seg_model.names[int(cls_id)]}")

                        #         self.destination = mask

                        #     elif self.obstacle == False and cls_id == self.center_objective and conf > 0.85:
                        #         self.avoid = False

                        #     elif cls_id == 0:
                        #         self.get_logger().info(f"I've got in {self.seg_model.names[int(cls_id)]}, seg_신뢰도: {conf:.2f}, 이전 차로: {self.seg_model.names[int(self.center_objective)]}")
                        #         self.center_objective = None
                        #         self.destination = mask
                        #         self.avoid = False
                    else:
                        
                        self.destination = [320, 640]

                    self.center.get_centroid(self.destination)
                    self.prev_center = self.destination

            processor = PurePursuit(self.destination, self.lookahead_distance)
            
            lookahead_distance, self.bezier_points = processor.get_bezier_points(self.car_position, (self.center.centroid_x, self.center.centroid_y))

            bezier_path = processor.bezier_curve(self.bezier_points)
            lookahead_point = processor.find_lookahead_point(bezier_path, self.car_position, lookahead_distance)
            # print(f"lookahead_point: {lookahead_point}")
            
            slope = self.get_slope(lookahead_point)
            try:
                if math.copysign(1,slope) != math.copysign(1,self.prev_slope) and np.abs(slope - self.prev_slope) > 10:
                    # self.get_logger().info(f"보정 전 slope: {slope:.3f} deg")
                    # self.get_logger().info(f"보정 후 slope: {self.prev_slope:.3f} deg")
                    slope = self.prev_slope
                    # print("-------------")
                else:
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
    seg_checkpoint_path = '/root/asap/data/seg_best3.pt'
    det_checkpoint_path = '/root/asap/data/det_best3.pt'

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
