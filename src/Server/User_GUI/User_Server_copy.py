import os 
import sys 
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import time 
import rclpy
import numpy as np 
import copy
import cv2
import face_recognition

from Thread import Thread, CarThread
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import *
from PyQt5 import uic
from PyQt5.QtCore import Qt

from Init_user_server import Init_User_Server
from DB.Car import Car
from DB.Member import Member
from Control_CAR import MyCar
from PathPlanner import PathPlanning
from Recognition import FaceRecognitionModel


class GetPosition(QDialog):
    def __init__(self, pixmap=None):
        super().__init__()
        self.pixmap = pixmap
        self.p_x, self.p_y = None, None
        
    def mousePressEvent(self, event):
        pos_area1 = self.ban_area_1.geometry()
        pos_area2 = self.ban_area_2.geometry()
        pos_area3 = self.ban_area_3.geometry()
        ban_areas = [[pos_area1.x(), pos_area1.y()-80, pos_area1.width()+pos_area1.x(), pos_area1.height() + pos_area1.y()-80],
                     [pos_area2.x(), pos_area2.y()-80, pos_area2.width()+pos_area2.x(), pos_area2.height() + pos_area2.y()-80],
                     [pos_area3.x(), pos_area3.y()-80, pos_area3.width()+pos_area3.x(), pos_area3.height() + pos_area3.y()-80]]
        
        if event.button() == Qt.LeftButton:
            real_x, real_y = event.x(), event.y()
            x, y = real_x-10 , real_y-90
            if x<0 or y<0:
                x, y = '', ''
            elif x>381 or y>541:
                x, y = '', ''
            if not(x=='' or y==''):
                for ban_area in ban_areas:
                    if (x>int(ban_area[0]) and x<int(ban_area[2])) and (y>int(ban_area[1]) and y<int(ban_area[3])):
                        x, y = '', ''
                        break
            
            self.pos_x.setText(str(x))
            self.pos_y.setText(str(y))
            self.map.setPixmap(self.pixmap)

            k = 7.445
            self.p_x, self.p_y = x, y

            if x=='' or y=='':
                x,y = 0,0
            painter = QPainter(self.map.pixmap())
            painter.setPen(QPen(Qt.blue, 5))
            painter.setBrush(QBrush(Qt.blue))
            painter.drawEllipse(int((x-8)*k), int((y-5)*k), 100, 100)
                    

class User_Server(Init_User_Server):
    def __init__(self):
        super().__init__()

        img_path = os.path.dirname(os.path.abspath(__file__)) + '/Images/map.jpg'
        self.pixmap = QPixmap(img_path)
        self.map_w, self.map_h = self.pixmap.width(), self.pixmap.height()

        self.init_uic()
        self.init_parameters()
        self.init_btns()
        self.set_Threads()

    def init_parameters(self):
        self.popup_window('main_window')
        self.is_start=False
        self.passwords = {}
        self.password = ''
        self.before_len = 0
        self.origin_x, self.origin_y = 215, 306
        self.goal_x, self.goal_y = None, None,
        self.all_path, self.waypoints = None, None
        self.is_arrive = False
        self.is_goal = False
        self.is_auto_driving = False
        self.USER_ID = None
        self.pp_cnt = False  

        self.pathplanner = PathPlanning()

        self.face_recognition_model = FaceRecognitionModel()
        
    def init_btns(self):
        self.btn_return.clicked.connect(self.checking_inside)
        self.btn_start.clicked.connect(self.btn_call_car)
        self.register_2.clicked.connect(self.register_member)
        self.btn_logout.clicked.connect(self.logout)
        self.btn_rent.clicked.connect(self.btn_call_car)
        self.btn_board.clicked.connect(self.checking_user)

        self.btn_back.clicked.connect(self.go_back)
        self.btn_home.clicked.connect(lambda: self.popup_window('main_window'))
        self.map_info.clicked.connect(lambda: self.popup_window('map'))
        self.car_info.clicked.connect(lambda: self.popup_window('car'))
        self.login.clicked.connect(lambda: self.popup_window('login'))
        self.PW.textChanged.connect(self.input_password)
        self.btn_enter.clicked.connect(self.click_enter)
        self.car_searchbar.textChanged.connect(self.search_car)
        self.car_select_info_type.currentIndexChanged.connect(self.search_car)
        
    def add_member(self):
        ID = self.register_window.id.text().strip()
        PW = self.register_window.pw.text().strip()
        PW_check = self.register_window.pw_check.text().strip()
        name = self.register_window.name.text().strip()
        phone = self.register_window.phone.text().strip()
        license = self.register_window.license.text().strip()
        img_path = self.register_window.img_path.text().strip()
        data = [ID, PW, PW_check, name, phone, license, img_path]

        if PW != PW_check:
            self.pw_alert.show()
            return
        elif '' in data:
            self.add_alert.show()
            return
        else:
            member = Member(name, phone, license, img_path)
            member.ID, member.PW = ID, PW
            self.memdb.add_member(member)
            self.popup_window('main_window')
            self.register_window.id.setText('')
            self.register_window.pw.setText('')
            self.register_window.pw_check.setText('')
            self.register_window.name.setText('')
            self.register_window.phone.setText('')
            self.register_window.license.setText('')
            self.register_window.img_path.setText('')
            self.register_window.close()

    def go_back(self):
        search_text = self.car_searchbar.text().strip()
        search_type = self.car_select_info_type.currentText().strip()
        try:
            self.car_name_listup(search_text, info_type=search_type)
        except:
            self.car_name_listup()

    def search_car(self):
        search_text = self.car_searchbar.text().strip()
        search_type = self.car_select_info_type.currentText().strip()
        
        if search_type == 'name':
            try:
                self.car_number_listup(car_name=search_text)
            except:
                self.car_name_listup()
        else:      
            self.car_name_listup(search_text, info_type=search_type)

    def init_uic(self):
        self.register_window = QDialog()
        uic.loadUi('./UI/register.ui', self.register_window)
        self.register_window.register_2.clicked.connect(self.add_member)
        self.register_window.add_image.clicked.connect(self.add_img)
        self.register_window.cancel.clicked.connect(lambda: self.close_window('register'))

        self.add_alert = QDialog()
        uic.loadUi('./UI/register_alert.ui', self.add_alert)
        self.add_alert.alert_ok.clicked.connect(lambda: self.close_window('alert'))

        self.id_alert = QDialog()
        uic.loadUi('./UI/id_alert.ui', self.id_alert)
        self.id_alert.alert_ok.clicked.connect(lambda: self.close_window('id_alert'))

        self.pw_alert = QDialog()
        uic.loadUi('./UI/password_alert.ui', self.pw_alert)
        self.pw_alert.alert_ok.clicked.connect(lambda: self.close_window('pw_alert'))

        self.checkinside = QDialog()
        uic.loadUi('./UI/return_check.ui', self.checkinside)
        self.checkinside.pushButton.clicked.connect(self.checking)
        # self.checkinside.btn_cancel.clicked.connect(lambda: self.close_window('return'))
        self.checkinside.btn_cancel.clicked.connect(self.checking_user)
        

        self.loginWindow = QDialog()
        uic.loadUi('./UI/login_alert.ui', self.loginWindow)
        self.loginWindow.login_ok.clicked.connect(lambda: self.close_window('login'))

        self.renting_window = QDialog()
        uic.loadUi('./UI/rurent.ui', self.renting_window)
        self.renting_window.btn_ok.clicked.connect(self.select_position)
        self.renting_window.btn_cancel.clicked.connect(lambda: self.close_window('renting'))

        self.position_window = GetPosition(self.pixmap)
        self.position_window.setFixedSize(401, 721)
        uic.loadUi('./UI/select_position.ui', self.position_window)
        self.position_window.btn_call.clicked.connect(self.renting_car)
        self.position_window.btn_cancel.clicked.connect(lambda: self.close_window('position'))

        self.arrive_window = QDialog()
        uic.loadUi('./UI/arrive.ui', self.arrive_window)
        self.arrive_window.btn_ok.clicked.connect(lambda: self.close_window('arrive'))


    def close_window(self, dialog):
        if dialog=='return':
            self.checkinside.close()
        elif dialog=='register':
            self.register_window.id.setText('')
            self.register_window.pw.setText('')
            self.register_window.pw_check.setText('')
            self.register_window.name.setText('')
            self.register_window.phone.setText('')
            self.register_window.license.setText('')
            self.register_window.close()
        elif dialog=='alert':
            self.add_alert.close()
        elif dialog=='pw_alert':
            self.pw_alert.close()
        elif dialog=='id_alert':
            self.id_alert.close()
        elif dialog=='login':
            self.loginWindow.close()   
        elif dialog=='renting':
            self.renting_window.close()
        elif dialog=='position':
            self.position_window.close()
            self.position_window.pos_x.setText('')
            self.position_window.pos_y.setText('')
            self.position_window.map.setPixmap(self.pixmap)
            self.position_window.p_x = 0
            self.position_window.p_y = 0
        elif dialog=='arrive':
            self.arrive_window.close()

    def add_img(self):
        img_path = QFileDialog.getOpenFileName(self, 'Open file', './', 'Image files (*.jpg *.png)')[0]
        self.register_window.img_path.setText(img_path)
        pixmap = QPixmap(img_path)
        pixmap = pixmap.scaled(300, 300, Qt.KeepAspectRatio, Qt.SmoothTransformation)

        self.register_window.image_frame.setPixmap(pixmap)

    def register_member(self):
        self.register_window.show()

    def logout(self):
        self.is_login = False
        self.ID.setText('')
        self.login.show()
        self.register_2.show()
        self.btn_logout.hide()        
        self.popup_window('main_window')

    def select_position(self):
        self.position_window.show()
        self.close_window('renting')

    def checking_user(self):
        # self.video = cv2.VideoCapture(-1)
        # self.face_thread.start()
        # self.face_thread.running = True
        # user = self.memdb.member_dict[self.memdb.IDPW[self.USER_ID][1]]
        user = self.memdb.member_dict[self.memdb.IDPW['pinky'][1]]
        self.face_recognition_model.set_user_image(user.img_path)
        self.face_recognition_model.set_known_user(user.img_path, user.name)

    def matching_face(self):
        ret, frame = self.video.read()

        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = cv2.flip(frame, 1)

            face_locations = face_recognition.face_locations(frame)
            face_encodings = face_recognition.face_encodings(frame, face_locations)

            face_names = []
            for face_encoding in face_encodings:
                matches = face_recognition.compare_faces(self.face_recognition_model.known_face_encodings, face_encoding)
                name = "Unknown"

                face_distances = face_recognition.face_distance(self.face_recognition_model.known_face_encodings, face_encoding)
                best_match_index = np.argmin(face_distances)

                if matches[best_match_index]:
                    name = self.face_recognition_model.known_face_names[best_match_index]

                face_names.append(name)

            for (top, right, bottom, left), face_name in zip(face_locations, face_names):
                top *= 4
                right *= 4
                bottom *= 4
                left *= 4

                cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
                cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
                font = cv2.FONT_HERSHEY_DUPLEX
                cv2.putText(frame, face_name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)



    def checking_inside(self):
        self.checkinside.show()
        
    def checking(self):
        self.is_renting = False
        self.car_thread.stop()
        self.myCar.destroy_node()
        self.battery_check.stop()
        self.map_thread.stop()
        self.cardb.update_values('car', 'is_rented=0', f'car_number="{str(self.rentcar_number)}"')
        rclpy.shutdown()
        self.close_window('return')
        self.popup_window('main_window')
        self.rentcar_number = None 
        self.all_path, self.waypoints = None, None
    
    def btn_call_car(self, data=None, info_type='all', mode='show'):
        self.popup_window('select_car')
        return self.car_name_listup(data, info_type, mode)
    
    def running_car(self):
        try:
            rclpy.init()
        except:
            pass

        self.myCar = MyCar()
        rclpy.spin(self.myCar)

    def set_arrive(self):
        self.is_arrive = False 
        self.all_path, self.waypoints = None, None
        self.arrive_window.show()


    def renting_car(self):
        self.is_renting = True 
        goal_x, goal_y = self.position_window.p_x, self.position_window.p_y
        w, h = self.position_window.map.width(), self.position_window.map.height()
        self.goal_x = goal_x*(self.origin_x/w)
        self.goal_y = goal_y*(self.origin_y/h)
        
        self.close_window('position')        
        self.cardb.update_values('car', 'is_rented=1', f'car_number="{str(self.rentcar_number)}"')
        self.close_window('renting')
        self.popup_window('map')

        car = self.cardb.car_dict[self.rentcar_number]
        ROS_DOMAIN_ID = car.pin_number

        os.environ['ROS_DOMAIN_ID'] = ROS_DOMAIN_ID
        self.car_thread = CarThread(target=self.running_car)
        self.car_thread.start()
        self.car_thread.running = True
        self.battery_check.start()
        self.battery_check.running = True
        self.map_thread.start()
        self.map_thread.running = True


    def update_battery(self):
        car = self.cardb.car_dict[self.rentcar_number]
        try:
            if self.myCar.battery == 0:
                battery = car.battery
            else:
                battery = self.myCar.battery
            if battery == None:
                battery = 0
        except:
            return

        self.cardb.update_values('car', f'battery={battery:.2f}', f'car_number="{str(self.rentcar_number)}"')
        car.battery = battery
        self.battery_percent.setText(f'{int(battery)}%')
        self.battery_percent.setStyleSheet('color: white; font-size: 15px')
        percent = int(((battery+0.00001)/100)*71)
        self.battery_green.resize(percent, 31)
        

    def show_rent_window(self, car_number):
        super().show_rent_window(car_number)
        self.renting_window.show()
        self.rentcar_number = car_number

        
    def popup_window(self, window_type):
        self.cardb.init_db()
        self.is_login=True
        if self.is_login or window_type=='main_window' or window_type=='login':
            for key, value in self.WINDOW_TYPES.items():
                if key == window_type:
                    if self.WINDOW_TYPES[key]==False:
                        self.OPEN_WINDOW[key]()

                    self.WINDOW_TYPES[key] = True
                else:
                    if self.WINDOW_TYPES[key]==True:
                        self.HIDE_WINDOW[key]()
                    self.WINDOW_TYPES[key] = False
        else:
            self.loginWindow.show()

    def click_enter(self):
        id = self.ID.text()

        if id == 'admin':
            if self.password == '1234':
                self.is_login = True
                self.popup_window("main_window")
            else:
                self.pw_alert.show()
                self.password = ''
                self.before_len = 0
                self.PW.setText('')
        else:
            if id in self.memdb.IDPW:
                if self.password == self.memdb.IDPW[id][0]:
                    self.USER_ID = id
                    self.is_login = True
                    self.password = ''
                    self.before_len = 0
                    self.popup_window("main_window")
                else:
                    self.pw_alert.show()
                    self.password = ''
                    self.before_len = 0
                    self.PW.setText('')
            else:
                self.id_alert.show()
                self.password = ''
                self.before_len = 0
                self.PW.setText('')
                self.ID.setText('')

    def input_password(self): 
        text = self.PW.text()
        length = len(text)
        
        if length == 0:
            self.before_len = 0
            self.password = ''
            return

        if text[-1] != '*' or length < self.before_len:
            if length >= self.before_len:
                self.password += text[-1]
                self.PW.setText('*'*(length))
            elif length < self.before_len:
                self.password = self.password[:-1]
            self.before_len = length

    def keyPressEvent(self, event):
        linear_x = 0.3
        angular_z = 1.
        if event.key() == Qt.Key_W: 
            self.myCar.msg.linear.x = linear_x
            self.myCar.msg.angular.z = 0.
        elif event.key() == Qt.Key_Q:
            self.myCar.msg.linear.x = linear_x
            self.myCar.msg.angular.z = angular_z
        elif event.key() == Qt.Key_E:
            self.myCar.msg.linear.x = linear_x
            self.myCar.msg.angular.z = -angular_z
        elif event.key() == Qt.Key_X:
            self.myCar.msg.linear.x = -linear_x
            self.myCar.msg.angular.z = 0.
        elif event.key() == Qt.Key_Z:
            self.myCar.msg.linear.x = -linear_x
            self.myCar.msg.angular.z = -angular_z
        elif event.key() == Qt.Key_C:
            self.myCar.msg.linear.x = -linear_x
            self.myCar.msg.angular.z = angular_z
        elif event.key() == Qt.Key_A:
            self.myCar.msg.angular.z = angular_z
        elif event.key() == Qt.Key_D:
            self.myCar.msg.angular.z = -angular_z
        elif event.key() == Qt.Key_S:
            self.myCar.msg.linear.x = 0.                        
            self.myCar.msg.angular.z = 0.                                           
        

    def set_Threads(self):
        self.battery_check = Thread(sec=3)
        self.battery_check.data.connect(self.update_battery)
        
        self.map_thread = Thread(sec=0.5)
        self.map_thread.data.connect(self.update_map)

        self.face_thread = Thread(sec=0.3)
        self.face_thread.data.connect(self.matching_face)

    def update_map(self):
        try:
            px, py = 0.84, 0.94
            data = self.myCar.pos 
            position = data.pose.position 
            orientation = data.pose.orientation
            
            z, w = orientation.z, orientation.w
            pos_x, pos_y = position.x, position.y
            
            self.cardb.update_values('car', f'pos="{pos_x}, {pos_y}"', f'car_number="{str(self.rentcar_number)}"')
            # pgm파일 x100배 한 좌표
            pos_x = (px + pos_x)*100-16
            pos_y = (self.origin_y - (py + pos_y)*100)-18

            plan_x, plan_y = int(pos_x*(183/(self.origin_x  - 16*2)))+5, int(pos_y*(270/(self.origin_y  - 18*2)))
            
            if self.all_path is None or self.waypoints is None:
                origin_x = 184
                origin_y = 270
                self.all_path, self.waypoints = self.pathplanner.generate_waypoint((plan_x, plan_y), (self.goal_x, self.goal_y), self.is_renting, self.pp_cnt)
                self.is_goal = True

                # 최초 경로 생성 후 True 로 바뀜.
                if self.all_path and self.waypoints:
                    self.pp_cnt = True

                for i in range(len(self.all_path)):
                    self.all_path[i][0] = int(self.all_path[i][0] * (-0.2 + self.map_w/origin_x))
                    self.all_path[i][1] = int((self.all_path[i][1]-2) * (0.6+self.map_h/origin_y))

            # 차량 반환 시 경로 생성 후 pp_cnt = False 로 초기화
            if self.is_renting == False:
                self.pp_cnt = False

            pos_x = int((pos_x) * (self.map_w/(self.origin_x  - 16*2)))
            pos_y = int((pos_y) * (self.map_h/(self.origin_y - 18*2)))
            goal_x = int((self.goal_x-4) * (self.map_w/self.origin_x))
            goal_y = int((self.goal_y-5) * (self.map_h/self.origin_y))

            self.all_path[0] = [pos_y+50, pos_x+50]
            self.all_path[-1] = [goal_y+50, goal_x+50]

            if self.is_renting or self.is_goal:
                self.map_frame.setPixmap(self.pixmap)
                painter = QPainter(self.map_frame.pixmap())
                for i in range(len(self.all_path)-1):
                    painter.setPen(QPen(Qt.green, 20))
                    painter.setBrush(QBrush(Qt.green))
                    cur_points = self.all_path[i]
                    next_points = self.all_path[i+1]
                    cur_x, cur_y = cur_points
                    next_x, next_y = next_points
                    painter.drawLine(cur_y, cur_x, next_y, next_x)

                painter.setPen(QPen(Qt.red, 5))
                painter.setBrush(QBrush(Qt.red))
                painter.drawEllipse(pos_x, pos_y, 100, 100)

                if not self.is_arrive:
                    painter.setPen(QPen(Qt.blue, 5))
                    painter.setBrush(QBrush(Qt.blue))
                    painter.drawEllipse(goal_x, goal_y, 100, 100)
                painter.end()
            else:
                for i in range(len(self.all_path)-1):
                    cur_points = self.all_path[i]
                    next_points = self.all_path[i+1]
                    cur_x, cur_y = cur_points
                    next_x, next_y = next_points

            if len(self.all_path) == 1:
                self.is_arrive = True
                self.is_goal = False
                self.myCar.waypoint.data = f'{pos_x, pos_y, self.all_path[0][1], self.all_path[0][0], z, w, self.is_arrive}'
                self.set_arrive()
            else:
                self.is_arrive = False
                if np.linalg.norm(self.all_path[0] - self.all_path[1]) < 200:
                    self.all_path = self.all_path[1:]
                self.myCar.waypoint.data = f'{pos_x, pos_y, self.all_path[1][1], self.all_path[1][0], z, w, self.is_arrive}'
        except:
            pass



if __name__ == "__main__":
    app = QApplication(sys.argv)    
    Window = User_Server()
    Window.show()
    sys.exit(app.exec_())
    window.close_connection()
