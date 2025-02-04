import os
import sys 
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import *
from PyQt5 import uic
from DB.Car import CarDB
from DB.Member import MemberDB

# 입력기 환경 변수 설정
os.environ["GTK_IM_MODULE"] = "fcitx"
os.environ["QT_IM_MODULE"] = "fcitx"
os.environ["XMODIFIERS"] = "@im=fcitx"
os.environ["QT_QPA_PLATFORMTHEME"] = "qt5ct"
ui_file = "./UI/Server.ui"
form_class = uic.loadUiType(ui_file)[0]

class CustomerTableModel(QAbstractTableModel):
    def __init__(self, data, headers):
        super().__init__()
        self.data = data  # 테이블 데이터
        self.headers = headers  # 열 이름

    def rowCount(self, parent=None):
        return len(self.data)

    def columnCount(self, parent=None):
        return len(self.headers)

    def data(self, index, role=Qt.DisplayRole):
        if role == Qt.DisplayRole or role == Qt.EditRole:
            return self.data[index.row()][index.column()]
        elif role == Qt.TextAlignmentRole:
            return Qt.AlignCenter  # 셀 데이터를 가운데 정렬
        return QVariant()

    def setData(self, index, value, role=Qt.EditRole):
        if role == Qt.EditRole:
            self.data[index.row()][index.column()] = value
            self.dataChanged.emit(index, index, [role])
            return True
        return False

    def flags(self, index):
        return Qt.ItemIsEditable | Qt.ItemIsEnabled | Qt.ItemIsSelectable

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                return self.headers[section]
            elif orientation == Qt.Vertical:
                return str(section + 1)
        elif role == Qt.TextAlignmentRole:  # 헤더도 정렬 가능
            return Qt.AlignCenter
        return QVariant()
    

class Init_Manage_Server(QMainWindow, form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setFixedSize(1161, 641)
        self.car_buttons = []
        self.cardb = CarDB()
        self.memdb = MemberDB()
        self.delete_car_info = None
        self.is_login = False 
        self.WINDOW_TYPES = {
            "main_window": False,
            "login": False,
            "car": False,
            "member":False,
            "map":False
        }
        self.OPEN_WINDOW = {
            "main_window": self.show_main_window,
            "login": self.show_login,
            "car": self.show_car,
            "member": self.show_member_window,
            "map": self.show_map,
        }
        self.HIDE_WINDOW = {
            "main_window": self.hide_main_window,
            "login": self.hide_login,
            "car": self.hide_car,
            "member": self.hide_member_window,
            "map": self.hide_map,
        }

    def initalize(self):
        self.set_member_table()
        self.init_memberlist()
        self.hide_login()
        self.hide_car()
        self.hide_member_window()
        self.hide_map()
        self.init_logo.show()
        self.init_login.show()
        self.init_logout.hide()
        self.car_scrollArea.setAlignment(Qt.AlignTop | Qt.AlignLeft) 
        self.car_select_info_type.clear()
        items = ['all', 'brand', 'type', 'name']
        self.car_select_info_type.addItems(items)
        self.member_select_type.clear()
        items = ['all', 'name', 'phone', 'member code']
        self.member_select_type.addItems(items)

        self.map_frame.setPixmap(QPixmap('./Image/map.jpg'))
        self.init_logo.setPixmap(QPixmap('./Image/ASAP_LOGO_long.jpg'))
        

    def set_member_table(self):
        self.table_data = []
        self.headers = ['이름', 
                        '휴대폰', 
                        '면허 번호',
                        '회원 번호',
                        '회원 등급',
                        '포인트',
                        '총 탑승 시간',
                        '랜트 여부',
                        '사고 이력']
        self.table_model = CustomerTableModel(self.table_data, self.headers)
        self.member_widget.setModel(self.table_model)
        header = self.member_widget.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)  # 열을 테이블 너비에 맞게 확장
        self.member_widget.verticalHeader().setVisible(False)
        

    def show_main_window(self):
        self.hide_login()
        self.init_logo.show()
        self.car_searchbar.setText('')
        self.car_select_info_type.setCurrentIndex(0)
        self.member_searchbar.setText('')
        self.member_select_type.setCurrentIndex(0)
        if self.is_login==False:
            self.init_login.show()
            self.init_logout.hide()
        else:
            self.init_login.hide()
            self.init_logout.show()
        
    def show_map(self):
        self.map_widget.show()
        
    
    def hide_map(self):
        self.map_widget.hide()


    def hide_main_window(self):
        self.init_logo.hide()
        self.init_login.hide()

    def hide_member_window(self):
        self.member_bg.hide()
        self.member_widget.hide()
        self.member_searchbar.hide()
        self.member_searchbar_bg.hide()
        self.member_search.hide()
        self.member_register.hide()
        self.member_remove.hide()
        self.member_select_type.hide()


    def show_member_window(self):
        self.member_bg.show()
        self.member_widget.show()
        self.member_searchbar.show()
        self.member_searchbar_bg.show()
        self.member_search.show()
        self.member_register.show()
        self.member_remove.show()
        self.member_select_type.show()

    def hide_login(self):
        self.login_bg.hide()
        self.login_id.hide()
        self.login_pw.hide()
        self.login_inid.hide()
        self.login_inpw.hide()
        self.login_enter.hide()

    def show_login(self):
        self.login_bg.show()
        self.login_id.show()
        self.login_pw.show()
        self.login_inid.show()
        self.login_inpw.show()
        self.login_enter.show()

    def hide_car(self):
        self.remove_car_box()
        self.car_bg.hide()
        self.car_searchbar.hide()
        self.car_searchbar_bg.hide()
        self.car_widget.hide()
        self.car_select_info_type.hide()
        self.car_show.hide()
        self.car_register.hide()
        self.car_remove.hide()
        self.car_delete.hide()
        self.car_delete_number.hide()
        self.car_back.hide()

    def show_car(self):
        self.car_name_listup()
        self.car_bg.show()
        self.car_searchbar.show()
        self.car_searchbar_bg.show()
        self.car_widget.show()
        self.car_show.show()
        self.car_select_info_type.show()
        self.car_register.show()

    def remove_car_box(self):
        for buttons in self.car_buttons:
            btn, description = buttons
            btn.hide()
            description.hide()
            btn.deleteLater()
            description.deleteLater()
        self.car_buttons = []

    def init_memberlist(self):
        self.table_model.data = []
        for member in self.memdb.member_dict.values():
            self.member_listup(member)

    def member_listup(self, member):
        name = member.name
        phone = member.phone
        licenses = member.license
        mem_code = member.member_code
        mem_grade = member.member_grade
        point = member.point
        is_renting = member.is_renting
        
        if len(member.rent_cost)==0:
            total_time = 0
        else:
            for val in member.values():
                pass 
        acidents = len(member.car_acident)
        data = [name, 
                phone, 
                licenses, 
                mem_code, 
                mem_grade, 
                point, 
                total_time, 
                is_renting, 
                acidents]
        self.table_model.data.append(data)
        self.table_model.layoutChanged.emit()

    def apply_filter(self):
        """검색 필터 적용"""
        search_type = self.member_select_type.currentText()
        search_text = self.member_searchbar.text().strip()
        if search_type == 'name':
            member_codes = sorted(self.memdb.get_data('person', 'member_code', f'name LIKE "%{search_text}%"'))
        elif search_type == 'phone':
            member_codes = sorted(self.memdb.get_data('person', 'member_code', f'phone LIKE "%{search_text}%"'))
        elif search_type == 'member_code':
            member_codes = sorted(self.memdb.get_data('person', 'member_code', f'member_code LIKE "%{search_text}%"'))  
        else:
            member_codes = sorted(self.memdb.get_data('person', 'member_code', f'member_code LIKE "%{search_text}%" OR name LIKE "%{search_text}%" OR phone LIKE "%{search_text}%" OR member_grade LIKE "%{search_text}%" OR car_acident LIKE "%{search_text}%"'))
            
        self.table_model.data = []
        for code in member_codes:
            member = self.memdb.member_dict[code[0]]
            self.member_listup(member)

    def car_number_listup(self, car_name=None, mode='show'):
        self.cardb.init_db()
        self.car_remove.show()
        self.car_back.show()
        self.remove_car_box()
        num_columns = 4 
        self.delete_car_info = car_name
        car_numbers = self.cardb.car_name[car_name]
        for i, car_number in enumerate(car_numbers): 
            i = i + 1  
            car = self.cardb.car_dict[car_number]          

            button = QPushButton()
            
            icon = QIcon(car.img_path) 
            button.setIcon(icon)
            button.setIconSize(QSize(200, 160)) 
            button.setFixedSize(224, 180) 
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
                button.clicked.connect(lambda _, n=car_number: self.show_cur_pos(n))

            
            rent = '대여중' if car.isrented else '대기중'
            rent = '파손' if car.destroied else rent
            if car.battery == None:
                battery = 0
            else:
                battery = int(car.battery)
            description = QLabel(f"{car.car_number}\n상태 : {rent}  배터리 : {battery}%")
            if rent == '대여중':
                description.setStyleSheet("font: 12px; color: gray; text-align: center;")
            elif rent == '대기중':
                description.setStyleSheet("font: 12px; color: green; text-align: center;")
            else:
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

    def show_cur_pos(self, car_number):
        car = self.cardb.car_dict[car_number]
        print(f"Car number : {car.car_number} \nBrand : {car.brand} \nCar name : {car.car_name} \nType : {car.type} \nPin number : {car.pin_number} \nBattery : {car.battery} \nDestroied : {car.destroied} \nIs rented : {car.isrented} \nImage path : {car.img_path} \nPosition : {car.pos}")
    
    def car_name_listup(self, data=None, info_type='all', mode='show'):
        self.cardb.init_db()
        self.remove_car_box()

        self.car_back.hide()
        self.car_delete.hide()
        self.car_remove.hide()
        self.car_delete_number.hide()

        num_columns = 4 

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
            button.setIconSize(QSize(200, 160)) 
            button.setFixedSize(224, 180) 
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
            
