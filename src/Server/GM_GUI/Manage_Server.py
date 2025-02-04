import os 
import sys 
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import time 
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import *
from PyQt5 import uic
from Init_manage_server import Init_Manage_Server
from Thread import Thread
from PyQt5.QtGui import QFont
from DB.Car import Car
from DB.Member import Member

ui_file = "./UI/Server.ui"
form_class = uic.loadUiType(ui_file)[0]

class Manage_Server(Init_Manage_Server, form_class):
    def __init__(self):
        super().__init__()
        self.initalize()
        self.init_parameters()
        self.set_Threads()
        self.button_tools()
        self.init_uic()
        img_path = os.path.dirname(os.path.abspath(__file__)) + '/Image/map.jpg'
        self.pixmap = QPixmap(img_path)
        self.map_w, self.map_h = self.pixmap.width(), self.pixmap.height()

# Main Window 관련
################################################################################################

    def button_tools(self):
        self.login_enter.clicked.connect(self.click_enter)
        self.init_logout.clicked.connect(self.logout)
        self.car_register.clicked.connect(self.add_car)
        self.member_register.clicked.connect(self.add_member_btn)

        self.car_back.clicked.connect(lambda: self.car_name_listup())
        self.btn_home.clicked.connect(lambda: self.popup_window("main_window"))
        self.init_login.clicked.connect(lambda: self.popup_window("login"))
        self.Manage_car.clicked.connect(lambda: self.popup_window("car"))
        self.Manage_member.clicked.connect(lambda: self.popup_window("member"))
        self.RT_loc_info.clicked.connect(lambda: self.popup_window("map"))


        self.car_remove.clicked.connect(self.car_delete_popup)
        self.car_delete.clicked.connect(self.car_delete_btn)
        self.car_show.clicked.connect(self.show_type_info)

        self.member_remove.clicked.connect(self.member_delele_btn)
        self.member_searchbar.textChanged.connect(self.apply_filter)
        

    def show_type_info(self):
        self.cardb.init_db()

        try:
            typei = self.car_select_info_type.currentText().strip()
            name = self.car_searchbar.text()
            self.car_name_listup(name, typei)
        except:
            return

    def member_delele_btn(self):
        selected_indexes = self.member_widget.selectionModel().selectedRows()
        
        for index in sorted(selected_indexes, reverse=True):
            self.memdb.del_member(self.memdb.member_dict[self.table_model.data[index.row()][3]])
            del self.table_model.data[index.row()]

        self.table_model.layoutChanged.emit()  # 모델 갱신
        self.member_widget.setModel(self.table_model)

    def car_delete_btn(self):
        car_number = self.car_delete_number.text().strip()
        car = self.cardb.car_dict[car_number]
        self.cardb.remove_car(car)
        
        self.car_delete_number.hide()
        self.car_delete.hide()

        if len(self.car_buttons) != 0:
            self.remove_car_box()
        try:
            self.car_number_listup(car_name=self.delete_car_info)
        except:
            self.car_name_listup()


    def car_delete_popup(self):
        self.car_delete_number.show()
        self.car_delete.show()
        self.car_number_listup(car_name=self.delete_car_info, mode='delete')

        
    def popup_window(self, window_type):
        self.cardb.init_db()
        self.is_login=True
        if self.is_login or window_type=='login' or window_type=='main_window':
        
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
            self.log_alert.show()

        if window_type == "map":
            self.map_thread.start() 
            self.map_thread.is_running = True
        else:
            self.map_thread.stop() 
            self.map_thread.is_running = False

    
    def init_parameters(self):
        self.enter = False
        self.password = ''
        self.before_len = 0

    def logout(self):
        self.is_login = False
        self.init_logout.hide()
        self.init_login.show()

    def click_enter(self):
        id = self.login_inid.text()

        if self.password == '1234' and id == 'admin':
            self.is_login = True
            self.popup_window("main_window")
        else:
            self.loginWindow.show()

        self.password = ''
        self.before_len = 0
        self.login_inid.setText('')
        self.login_inpw.setText('')

    def login(self):
        self.init_logo.hide()
        self.init_login.hide()
        self.show_login()

    def set_Threads(self):
        self.login_inpw.textChanged.connect(self.input_password)

        self.map_thread = Thread(sec=1)
        self.map_thread.data.connect(self.update_map)

    def update_map(self):
        # map_frame = 
        self.cardb.init_db()
        car_informations = self.cardb.get_data('car', 'car_number, pos', 'is_rented=1')
        self.map_frame.setPixmap(self.pixmap)
        try:
            origin_x, origin_y = 215, 306
            px, py = 0.84, 0.94
            
            painter = QPainter(self.map_frame.pixmap())
            for car_info in car_informations:
                car_number, pos = car_info
                pos = pos.split(',')
                pos_x, pos_y = float(pos[0]), float(pos[1])

                pos_x = int((px + pos_x-0.1)*100 * (self.map_h/origin_x))
                pos_y = int(((py + pos_y-0.08)*100) * (self.map_w/origin_y))
                pos_x, pos_y = pos_y, pos_x
                print(pos_x, pos_y)
                painter.setPen(QPen(Qt.red, 5))
                painter.setBrush(QBrush(Qt.red))
                painter.drawEllipse(pos_x, pos_y, 100, 100)
            painter.end()
        except:
            pass

        total = len(self.cardb.car_dict)
        len_stay = 0
        len_rent = 0
        len_destroid = 0

        for car in self.cardb.car_dict.values():
            if car.isrented == 1:
                len_rent += 1
            if car.destroied == 1:
                len_destroid += 1
        len_stay = total - len_rent - len_destroid

        self.map_stay.setText(str(len_stay))
        self.map_rent.setText(str(len_rent))
        self.map_destroid.setText(str(len_destroid))

    def start_Threads(self):
        pass    

    def stop_Threads(self):
        pass

################################################################################################


# Popup window 관련
################################################################################################

    def init_uic(self):
        self.loginWindow = QDialog()
        uic.loadUi('./UI/login.ui', self.loginWindow)
        self.loginWindow.Check.clicked.connect(lambda: self.close_window('login'))

        self.add_carWindow = QDialog()
        uic.loadUi('./UI/add_car.ui', self.add_carWindow)
        self.add_carWindow.add_cancel.clicked.connect(lambda: self.close_window('add_car'))
        self.add_carWindow.add_register.clicked.connect(self.add_car_register)
        self.add_carWindow.add_image.clicked.connect(self.add_car_load_img)

        self.add_alert = QDialog()
        uic.loadUi('./UI/register_alert.ui', self.add_alert)
        self.add_alert.alert_ok.clicked.connect(lambda: self.close_window('alert'))

        self.log_alert = QDialog()
        uic.loadUi('./UI/login_alert.ui', self.log_alert)
        self.log_alert.login_ok.clicked.connect(lambda: self.close_window('login_alert'))

        self.add_member = QDialog()
        uic.loadUi("./UI/register.ui", self.add_member)
        self.add_member.cancel.clicked.connect(lambda: self.close_window('add_member'))
        self.add_member.register_2.clicked.connect(self.add_member_register)
        self.add_member.add_image.clicked.connect(self.add_member_load_img)

        self.pw_alert = QDialog()
        uic.loadUi('./UI/password_alert.ui', self.pw_alert)
        self.pw_alert.alert_ok.clicked.connect(lambda: self.close_window('pw_alert'))

    def close_window(self, dialog):
        if dialog == 'login':
            self.loginWindow.close()
        elif dialog == 'add_car':
            self.add_carWindow.close()
        elif dialog == 'alert':
            self.add_alert.close()
        elif dialog == 'login_alert':
            self.log_alert.close()
        elif dialog == 'add_member':
            self.add_member.close()
        elif dialog == 'pw_alert':
            self.pw_alert.close()

    def add_member_btn(self):
        self.add_member.show()


    def add_member_load_img(self):
        img_path = QFileDialog.getOpenFileName(self, 'Open file', './', 'Image files (*.jpg *.png)')[0]
        self.add_member.img_path.setText(img_path)
        pixmap = QPixmap(img_path)
        pixmap = pixmap.scaled(300, 300, Qt.KeepAspectRatio, Qt.SmoothTransformation)

        self.add_member.image_frame.setPixmap(pixmap)    


    def add_member_register(self):
        ID = self.add_member.id.text().strip()
        PW = self.add_member.pw.text().strip()
        PW_check = self.add_member.pw_check.text().strip()
        name = self.add_member.name.text().strip()
        phone = self.add_member.phone.text().strip()
        license = self.add_member.license.text().strip()
        img_path = self.add_member.img_path.text().strip()
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
            self.member_listup(member)
            self.add_member.id.setText('')
            self.add_member.pw.setText('')
            self.add_member.pw_check.setText('')
            self.add_member.name.setText('')
            self.add_member.phone.setText('')
            self.add_member.license.setText('')
            self.add_member.img_path.setText('')
            self.add_member.close()
        
    def add_car(self):
        self.add_carWindow.show()

    def add_car_load_img(self):
        img_path = QFileDialog.getOpenFileName(self, 'Open file', './', 'Image files (*.jpg *.png)')[0]
        self.add_carWindow.add_img_path.setText(img_path)
        pixmap = QPixmap(img_path)
        pixmap = pixmap.scaled(300, 300, Qt.KeepAspectRatio, Qt.SmoothTransformation)

        self.add_carWindow.add_image_frame.setPixmap(pixmap)


    def add_car_register(self):
        self.cardb.init_db()

        car_number = self.add_carWindow.add_number.text().strip()
        brand = self.add_carWindow.add_brand.text().strip()
        car_name = self.add_carWindow.add_name.text().strip()
        type = self.add_carWindow.add_type.text().strip()
        pin_number = self.add_carWindow.add_pin.text().strip()
        img_path = self.add_carWindow.add_img_path.text().strip()
        data = [car_number, brand, car_name, type, pin_number]
        if '' in data:
            self.add_alert.show()
            return
        else:
            car = Car(brand, car_name, type, car_number, pin_number, img_path)
            self.cardb.add_car(car)
            if len(self.car_buttons) != 0:
                self.remove_car_box()
            self.car_name_listup()
            self.add_carWindow.add_number.setText('')
            self.add_carWindow.add_brand.setText('')
            self.add_carWindow.add_name.setText('')
            self.add_carWindow.add_type.setText('')
            self.add_carWindow.add_pin.setText('')
            self.add_carWindow.add_img_path.setText('')
            self.add_carWindow.add_image_frame.clear()

            self.add_carWindow.close()


    def input_password(self): 
        text = self.login_inpw.text()
        length = len(text)
        
        if length == 0:
            self.before_len = 0
            self.password = ''
            return

        if text[-1] != '*' or length < self.before_len:
            if length >= self.before_len:
                self.password += text[-1]
                self.login_inpw.setText('*'*(length))
            elif length < self.before_len:
                self.password = self.password[:-1]
            self.before_len = length


################################################################################################





if __name__ == "__main__":
    app = QApplication(sys.argv)
    font = QFont("Nanum Gothic")  # Windows 환경에서는 'Malgun Gothic' 권장
    app.setFont(font)
    Window = Manage_Server()
    Window.show()
    sys.exit(app.exec_())
    window.close_connection()