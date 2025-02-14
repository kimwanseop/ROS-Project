import os 
import sys 
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import time 
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import *
from PyQt5 import uic
from DB.Car import CarDB
from DB.Member import MemberDB

ui_file = "./UI/User2.ui"
form_class = uic.loadUiType(ui_file)[0]

class Init_User_Server(QMainWindow, form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setFixedSize(401, 671)
        self.initalize()
        self.car_buttons = []
        self.WINDOW_TYPES = {
            'main_window': False,
            'map': False,
            'car' : False,
            'select_car' : False,
            'login' : False,
            # 'setting' : False 
        }
        self.OPEN_WINDOW = {
            'main_window': self.show_main_window,    
            'map':self.show_map_window,        
            'car':self.show_car_window,
            'select_car' : self.show_select_car_window,
            'login' : self.show_login_window,
        }
        self.HIDE_WINDOW = {
            'main_window': self.hide_main_window,
            'map':self.hide_map_window,
            'car':self.hide_car_window,        
            'select_car' : self.hide_select_car_window,
            'login' : self.hide_login_window,
        }
        self.cardb = CarDB()
        self.memdb = MemberDB()
    
    def initalize(self):
        
        self.is_login=False 
        self.is_renting = False
        self.is_boarding = False
        self.is_auther_check = False
        self.map_frame.setPixmap(QPixmap("./Images/map.jpg"))
        self.show_main_window() 
        self.hide_map_window()
        self.hide_car_window()
        self.hide_select_car_window()
        self.hide_login_window()
        self.car_select_info_type.clear()
        items = ['all', 'brand', 'type', 'name']
        self.car_select_info_type.addItems(items)

    def show_main_window(self):
        self.start_widget.show()
        self.btn_home.hide()
        self.car_searchbar.setText('')
        self.car_select_info_type.setCurrentIndex(0)
        if self.is_login:
            self.login.hide()
            self.register_2.hide()
            self.btn_logout.show()
        else:
            self.login.show()
            self.register_2.show()
            self.btn_logout.hide()

        if self.is_auther_check:
            self.btn_start.hide()
            self.label_start.hide()
            self.btn_boarding.show()
            self.label_boarding.show()
        else:
            self.btn_start.show()
            self.label_start.show()
            self.btn_boarding.hide()
            self.label_boarding.hide()
        
        if self.is_renting:
            self.btn_boarding.show()
            self.label_boarding.show()
            self.btn_start.hide()
            self.label_start.hide()
        else:
            self.btn_start.show()
            self.label_start.show()
            self.btn_boarding.hide()
            self.label_boarding.hide()


    def hide_main_window(self):
        self.start_widget.hide()
        self.login.hide()
        self.register_2.hide()
        self.btn_home.show()

    def show_select_car_window(self):
        self.cardb.init_db()
        self.select_car_widget.show()

    def hide_select_car_window(self):
        self.select_car_widget.hide()
    
    def hide_login_window(self):
        self.login_widget.hide()

    def show_login_window(self):
        self.login_widget.show()
        self.btn_home.show()
    
    def show_map_window(self):
        self.map_frame.setPixmap(QPixmap("./Images/map.jpg"))
        self.map_widget.show()
        self.btn_home.show()
        if self.is_renting:
            self.btn_return.show()
            self.btn_rent.hide()
            self.battery_bg.show()
            self.battery_green.show()
            self.battery_percent.show()
            self.select_mode.show()
            if self.is_boarding:
                self.btn_boarding.show()
                self.btn_home.setText('Get Off')
                # self.is_boarding = False
            else:
                self.btn_boarding.hide()
                self.btn_home.setText('Home')
            # self.map_thread.start()
        else:
            self.btn_home.setText('Home')
            self.select_mode.hide()
            self.btn_return.hide()
            self.btn_rent.show()
            self.battery_bg.hide()
            self.battery_green.hide()
            self.battery_percent.hide()
            

    def hide_map_window(self):
        self.map_widget.hide()
        

    def show_car_window(self):
        self.car_widget.show()

    def hide_car_window(self):
        self.car_widget.hide()


    def car_number_listup(self, car_name=None, mode='show'):
        self.cardb.init_db()
        self.btn_back.show()
        self.remove_car_box()
        num_columns = 3 
        self.delete_car_info = car_name
        car_numbers = self.cardb.car_name[car_name]
        for i, car_number in enumerate(car_numbers): 
            i = i + 1  
            car = self.cardb.car_dict[car_number]          

            button = QPushButton()
            
            icon = QIcon(car.img_path) 
            button.setIcon(icon)
            button.setIconSize(QSize(100, 80)) 
            button.setFixedSize(112, 90) 
            button.setStyleSheet('''
                                 font: bold 20px; 
                                 padding: 5px;
                                 background-color: white;
                                 border-radius: 10px;
                                 border: 1px solid black;
                                 text-align: top;
                                 ''' )
            if mode == 'delete':
                button.clicked.connect(lambda _, n=car_number: self.on_button_click(n, mode))
            else:
                button.clicked.connect(lambda _, n=car_number: self.show_rent_window(n))

            rent = '대여중' if car.isrented else '대기중'
            rent = '파손' if car.destroied else rent

            # 설명 생성
            if car.battery == None:
                battery = 0
            else:
                battery = int(car.battery)
            description = QLabel(f"{car.car_number}\n상태 : {rent}\n배터리 : {battery}%")
            if rent == '대여중':
                button.setEnabled(False)
                description.setStyleSheet("font: 12px; color: gray; text-align: center;")
            elif rent == '대기중':
                button.setEnabled(True)
                description.setStyleSheet("font: 12px; color: green; text-align: center;")
            else:
                button.setEnabled(False)
                description.setStyleSheet("font: 12px; color: red; text-align: center;")
            
            
            
            description.setAlignment(Qt.AlignCenter)
            
            # 버튼과 설명을 세로로 배치할 레이아웃
            button_with_description = QWidget()
            vbox = QVBoxLayout(button_with_description)
            vbox.addWidget(button)
            vbox.addWidget(description)
            vbox.setAlignment(Qt.AlignCenter)
            vbox.setContentsMargins(0, 0, 0, 0)  
            self.car_buttons.append([button, description])

            # 그리드에 추가
            row = (i - 1) // num_columns
            col = (i - 1) % num_columns
            self.car_scrollArea.addWidget(button_with_description, row, col)

    def show_rent_window(self, car_number):
        pass 
        

    def remove_car_box(self):
        for buttons in self.car_buttons:
            btn, description = buttons
            btn.hide()
            description.hide()
            btn.deleteLater()
            description.deleteLater()
        self.car_buttons = []    

    def car_name_listup(self, data=None, info_type='all', mode='show'):
        self.cardb.init_db()
        self.remove_car_box()
        self.btn_back.hide()


        num_columns = 3 

        car_info, car_name = self.cardb.get_stat_info(info_type)
        if len(self.cardb.car_dict)==0:
            return
        
        if info_type=='brand':
            car_name = {}
            for key in car_info:
                car_num_list = car_info[key]
                for car_num in car_num_list:
                    car = self.cardb.car_dict[car_num]
                    nm = car.car_name
                    brand = car.brand
                    if brand == data:
                        if nm not in car_name:
                            car_name[nm] = [car_num] 
                        else:
                            car_name[nm].append(car_num)
        elif info_type=='type':
            car_name = {}
            for key in car_info:
                car_num_list = car_info[key]
                for car_num in car_num_list:
                    car = self.cardb.car_dict[car_num]
                    nm = car.car_name
                    brand = car.type
                    if brand == data:
                        if nm not in car_name:
                            car_name[nm] = [car_num]
                        else:
                            car_name[nm].append(car_num)

        for i, name in enumerate(car_name): 
            i = i + 1
            total_num = len(car_name[name])
            rented_num = [self.cardb.car_dict[str(i)].isrented for i in car_name[name]].count(True)
            stay_num = total_num - rented_num

            button = QPushButton()
            
            car = self.cardb.car_dict[car_name[name][0]]
            icon = QIcon(car.img_path) 
            button.setIcon(icon)
            button.setIconSize(QSize(100, 80)) 
            button.setFixedSize(112, 90) 
            button.setStyleSheet('''
                                 font: bold 20px; 
                                 padding: 5px;
                                 background-color: white;
                                 border-radius: 10px;
                                 border: 1px solid black;
                                 text-align: top;
                                 ''' )
            
            button.clicked.connect(lambda _, n=name: self.car_number_listup(n, mode))

            # 설명 생성
            description = QLabel(f"{name}\n잔여량 : {stay_num} 대여량 : {rented_num}")
            description.setStyleSheet("font: 12px; color: black; text-align: center;")
            description.setAlignment(Qt.AlignCenter)
            
            # 버튼과 설명을 세로로 배치할 레이아웃
            button_with_description = QWidget()
            vbox = QVBoxLayout(button_with_description)
            vbox.addWidget(button)
            vbox.addWidget(description)
            vbox.setAlignment(Qt.AlignCenter)
            vbox.setContentsMargins(0, 0, 0, 0)  
            self.car_buttons.append([button, description])

            # 그리드에 추가
            row = (i - 1) // num_columns
            col = (i - 1) % num_columns
            self.car_scrollArea.addWidget(button_with_description, row, col)

    def on_button_click(self, car_number, mode):
        """버튼 클릭 이벤트 핸들러"""
        if mode == 'show':
            print(f"Button {car_number} clicked!")
        elif mode == 'delete':
            self.car_delete_number.setText(car_number)