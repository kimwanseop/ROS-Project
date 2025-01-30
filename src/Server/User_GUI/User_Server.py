import os 
import sys 
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import time 
from Thread import Thread
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import *
from PyQt5 import uic
from Init_user_server import Init_User_Server
from DB.Car import Car
from DB.Member import Member


class User_Server(Init_User_Server):
    def __init__(self):
        super().__init__()
        self.init_uic()
        self.init_parameters()
        self.init_btns()

    def init_parameters(self):
        self.popup_window('main_window')
        self.is_start=False
        self.passwords = {}
        self.password = ''
        self.before_len = 0
        

    def init_btns(self):
        self.btn_return.clicked.connect(self.checking_inside)
        # self.btn_start.clicked.connect(self.btn_call_car)
        self.register_2.clicked.connect(self.register_member)
        self.btn_logout.clicked.connect(self.logout)

        self.btn_home.clicked.connect(lambda: self.popup_window('main_window'))
        self.map_info.clicked.connect(lambda: self.popup_window('map'))
        self.car_info.clicked.connect(lambda: self.popup_window('car'))
        self.login.clicked.connect(lambda: self.popup_window('login'))
        self.PW.textChanged.connect(self.input_password)
        self.btn_enter.clicked.connect(self.click_enter)

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

        self.loginWindow = QDialog()
        uic.loadUi('./UI/login_alert.ui', self.loginWindow)
        self.loginWindow.login_ok.clicked.connect(lambda: self.close_window('login'))

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
            self.register_window.img_path.setText('')
            self.register_window.close()
        elif dialog=='alert':
            self.add_alert.close()
        elif dialog=='pw_alert':
            self.pw_alert.close()
        elif dialog=='id_alert':
            self.id_alert.close()
        elif dialog=='login':
            self.loginWindow.close()

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

    def checking_inside(self):
        self.checkinside.show()
        

    def checking(self):
        # self.checkinside2 = QDialog()
        # uic.loadUi('./UI/checking.ui', self.checkinside2)
        # self.checkinside2.show()
        
        # self.checkinside2.close()
        self.close_window('return')
        self.popup_window('main_window')
    
    def btn_call_car(self, data=None, info_type='all', mode='show'):
        self.popup_window('select_car')
        return self.car_name_listup(data, info_type, mode)


        
    def popup_window(self, window_type):
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
                if self.password == self.memdb.IDPW[id]:
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




if __name__ == "__main__":
    app = QApplication(sys.argv)
    Window = User_Server()
    Window.show()
    sys.exit(app.exec_())
    window.close_connection()