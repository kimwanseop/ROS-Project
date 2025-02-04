import time 
import threading
import rclpy as rp 
import matplotlib.pyplot as plt
import cv2
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped


class MyNode(Node):
    def __init__(self):
        super().__init__('mycar_publisher')
        self.create_subscription(PoseStamped, '/tracked_pose', self.battery_callback, 10)
        self.msg = None 

    def battery_callback(self, msg):
        self.msg = msg

class CarThread(threading.Thread):
    def __init__(self, target):
        super().__init__()
        self.target_function = target  
        self.running = True

    def run(self):
        while self.running: 
            self.target_function() 

    def stop(self):
        self.running = False 

class Run_Node():
    def __init__(self):
        self.img = cv2.imread('/root/ros-repo-3/src/Server/DB/map/asap_map_resized.pgm')
        self.mynode = MyNode()
        print(self.mynode.msg)

    def run_node(self):
        rp.spin(self.mynode)

    def start_thread(self):
        self.mythread = threading.Thread(target=self.run_node)
        self.mythread.start()

    def run(self):

        while True:
            try:
                position = self.mynode.msg.pose.position 
                orientation = self.mynode.msg.pose.orientation
                pos_x, pos_y = position.x, position.y
                print(f"pos_x : {pos_x} pos_y : {pos_y}")
                cv2.circle(self.img, (pos_x*100, pos_y*100), 10, (0, 0, 255), -1)
                cv2.imshow('img', self.img)
                plt.imswhow('img', self.img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except:
                pass
            time.sleep(1)

def main(args=None):
    rp.init()
    run_node = Run_Node()
    run_node.start_thread()
    run_node.run()
    run_node.mynode.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()



