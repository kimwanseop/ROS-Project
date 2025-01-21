import sys 
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import *
from PyQt5 import uic

ui_file = "./Manage_Server.ui"
form_class = uic.loadUiType(ui_file)[0]

class Init_Manage_Server(QMainWindow, form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.is_login = False 
        self.WINDOW_TYPES = {
            "main_window": False,
            "login": False,
        }
        self.OPEN_WINDOW = {
            "main_window": self.show_main_window,
            "login": self.show_login,
        }
        self.HIDE_WINDOW = {
            "main_window": self.hide_main_window,
            "login": self.hide_login,
        }
    def initalize(self):
        self.hide_login()
        self.init_logo.show()
        self.init_login.show()
        self.init_logout.hide()

    def show_main_window(self):
        self.hide_login()
        self.init_logo.show()
        if self.is_login==False:
            self.init_login.show()
            self.init_logout.hide()
        else:
            self.init_login.hide()
            self.init_logout.show()
        

    def hide_main_window(self):
        self.init_logo.hide()
        self.init_login.hide()

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

