import os 
import sys 
import time 
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import *
from PyQt5 import uic
from Init_manage_server import Init_Manage_Server
from Thread import Thread
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

ui_file = "./Manage_Server.ui"
form_class = uic.loadUiType(ui_file)[0]

class Manage_Server(Init_Manage_Server, form_class):
    def __init__(self):
        super().__init__()
        self.initalize()
        self.init_parameters()
        self.set_Threads()

        self.login_enter.clicked.connect(self.click_enter)
        self.init_logout.clicked.connect(self.logout)

        self.btn_home.clicked.connect(lambda: self.popup_window("main_window"))
        self.init_login.clicked.connect(lambda: self.popup_window("login"))

        self.loginWindow = QDialog()
        uic.loadUi('./login.ui', self.loginWindow)
        self.loginWindow.Check.clicked.connect(self.close_window)

    def close_window(self):
        self.loginWindow.close()

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

    def popup_window(self, window_type):
        for key, value in self.WINDOW_TYPES.items():
            if key == window_type:
                if self.WINDOW_TYPES[key]==False:
                    self.OPEN_WINDOW[key]()
                self.WINDOW_TYPES[key] = True
            else:
                if self.WINDOW_TYPES[key]==True:
                    self.HIDE_WINDOW[key]()
                self.WINDOW_TYPES[key] = False

        if window_type == "login":
            self.pw.start()

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

if __name__ == "__main__":
    app = QApplication(sys.argv)
    Window = Manage_Server()
    Window.show()
    sys.exit(app.exec_())
    window.close_connection()