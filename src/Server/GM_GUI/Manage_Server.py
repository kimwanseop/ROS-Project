import os 
import sys 
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
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

ui_file = "./Server.ui"
form_class = uic.loadUiType(ui_file)[0]

class Manage_Server(Init_Manage_Server, form_class):
    def __init__(self):
        super().__init__()
        self.initalize()
        self.init_parameters()
        self.set_Threads()
        self.button_tools()
        self.init_uic()
        self.init_car()

    def init_car(self):
        self.cardb.add_car(Car('Toyota', 'Corolla', '중형', '137가 1327', '13'))
        self.cardb.add_car(Car('Hyundai', 'Sonata', '중형', '247다 5328', '14'))
        self.cardb.add_car(Car('Kia', 'K5', '중형', '147가 1327', '15'))
        self.cardb.add_car(Car('BMW', 'M3', '대형', '137가 3287', '16'))
        self.cardb.add_car(Car('Hyundai', 'E300', '소형', '137가 1328', '17'))
        self.cardb.add_car(Car('Hyundai', 'E300', '소형', '137가 1329', '18'))
        self.cardb.add_car(Car('Hyundai', 'Genesis', '중형', '137가 1330', '19'))
        self.cardb.add_car(Car('Hyundai', 'Genesis', '중형', '137가 1331', '20'))

        self.cardb.add_car(Car('ASAP', 'Pinky', '소형', '324조 3321', '44'))
        self.cardb.add_car(Car('ASAP', 'Pinky', '소형', '321조 3322', '45'))
        self.cardb.add_car(Car('ASAP', 'Pinky', '소형', '321조 3323', '46'))
        self.cardb.add_car(Car('ASAP', 'Pinky', '소형', '321조 3324', '47'))
        self.cardb.add_car(Car('ASAP', 'Violet', '중형', '321조 3325', '48'))
        self.cardb.add_car(Car('ASAP', 'Violet', '중형', '321조 3326', '49'))
        self.cardb.add_car(Car('ASAP', 'Violet', '중형', '321조 3327', '50'))
        self.cardb.add_car(Car('ASAP', 'Minty', '대형', '321조 3328', '51'))
        self.cardb.add_car(Car('ASAP', 'Minty', '대형', '321조 3329', '52'))


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
        self.proxy_model = QSortFilterProxyModel()
        self.proxy_model.setSourceModel(self.table_model)
        self.proxy_model.setFilterKeyColumn(-1)  # 모든 열을 필터링 대상으로 설정
        self.proxy_model.setFilterCaseSensitivity(Qt.CaseInsensitive)  # 대소문자 구분 안 함
        self.member_widget.setModel(self.proxy_model)

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

        if window_type == "login":
            self.pw.start()

    def init_parameters(self):
        self.enter = False
        self.password = ''
        self.before_len = 0

    def logout(self):
        self.is_login = False
        self.init_logout.hide()
        self.init_login.show()

    def click_enter(self):
        self.enter = True

    def login(self):
        self.init_logo.hide()
        self.init_login.hide()
        self.show_login()

    def set_Threads(self):
        self.pw = Thread()
        self.pw.data.connect(self.input_password)

    def start_Threads(self):
        pass    

    def stop_Threads(self):
        pass

################################################################################################


# Popup window 관련
################################################################################################

    def init_uic(self):
        self.loginWindow = QDialog()
        uic.loadUi('./login.ui', self.loginWindow)
        self.loginWindow.Check.clicked.connect(lambda: self.close_window('login'))

        self.add_carWindow = QDialog()
        uic.loadUi('./add_car.ui', self.add_carWindow)
        self.add_carWindow.add_cancel.clicked.connect(lambda: self.close_window('add_car'))
        self.add_carWindow.add_register.clicked.connect(self.add_car_register)
        self.add_carWindow.add_image.clicked.connect(self.add_car_load_img)

        self.add_alert = QDialog()
        uic.loadUi('./register_alert.ui', self.add_alert)
        self.add_alert.alert_ok.clicked.connect(lambda: self.close_window('alert'))

        self.log_alert = QDialog()
        uic.loadUi('./login_alert.ui', self.log_alert)
        self.log_alert.login_ok.clicked.connect(lambda: self.close_window('login_alert'))

        self.add_member = QDialog()
        uic.loadUi("./user.ui", self.add_member)
        self.add_member.add_cancel.clicked.connect(lambda: self.close_window('add_member'))
        self.add_member.add_register.clicked.connect(self.add_member_register)
        self.add_member.add_image.clicked.connect(self.add_member_load_img)

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

    def add_member_btn(self):
        self.add_member.show()


    def add_member_load_img(self):
        img_path = QFileDialog.getOpenFileName(self, 'Open file', './', 'Image files (*.jpg *.png)')[0]
        self.add_member.add_img_path.setText(img_path)
        pixmap = QPixmap(img_path)
        pixmap = pixmap.scaled(300, 300, Qt.KeepAspectRatio, Qt.SmoothTransformation)

        self.add_member.add_image_frame.setPixmap(pixmap)    

    def add_member_register(self):
        add_name = self.add_member.add_name.text().strip()
        add_phone = self.add_member.add_phone.text().strip()
        add_license = self.add_member.add_license.text().strip()
        add_img_path = self.add_member.add_img_path.text().strip()
        data = [add_name, add_phone, add_license, add_img_path]
        if '' in data:
            self.add_alert.show()
            return
        else:
            member = Member(add_name, add_phone, add_license, add_img_path)
            self.memdb.add_member(member)
            self.member_listup(member)
            self.add_member.add_name.setText('')
            self.add_member.add_phone.setText('')
            self.add_member.add_license.setText('')
            self.add_member.add_img_path.setText('')
            self.add_member.add_image_frame.clear()
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
    '''
    123가 3245 소형
    '''
    def input_password(self): 
        text = self.login_inpw.text()
        length = len(text)

        if length == 0:
            return
        
        if length != self.before_len:
            self.login_inpw.setText('*'*(length-1) + text[-1])
            self.password += text[-1]
            self.before_len = length

        if self.enter:
            self.enter = False
            id = self.login_inid.text()

            if self.password == '1234' and id == 'admin':
                self.is_login = True
                self.pw.is_running = False
                self.pw.stop()
                self.popup_window("main_window")
            else:
                self.loginWindow.show()

            self.password = ''
            self.before_len = 0
            self.login_inid.setText('')
            self.login_inpw.setText('')

################################################################################################





if __name__ == "__main__":
    app = QApplication(sys.argv)
    font = QFont("Nanum Gothic")  # Windows 환경에서는 'Malgun Gothic' 권장
    app.setFont(font)
    Window = Manage_Server()
    Window.show()
    sys.exit(app.exec_())
    window.close_connection()