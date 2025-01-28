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


class User_Server(Init_User_Server):
    def __init__(self):
        super().__init__()
        self.is_start=False
        self.btn_return.clicked.connect(self.checking_inside)
        # self.btn_start.clicked.connect(self.get_car)

        self.map_info.clicked.connect(lambda: self.popup_window('map'))
        self.car_info.clicked.connect(lambda: self.popup_window('car'))

    
    def checking_inside(self):
        self.checkinside = QDialog()
        uic.loadUi('./UI/return_check.ui', self.checkinside)
        self.checkinside.show()
        self.checkinside.pushButton.clicked.connect(self.checking)
        

    def checking(self):
        # self.checkinside2 = QDialog()
        # uic.loadUi('./UI/checking.ui', self.checkinside2)
        # self.checkinside2.show()
        
        # self.checkinside2.close()
        self.close_window('return')
        self.popup_window('main_window')
    
    

    def close_window(self, dialog):
        if dialog=='return':
            self.checkinside.close()

        
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



if __name__ == "__main__":
    app = QApplication(sys.argv)
    Window = User_Server()
    Window.show()
    sys.exit(app.exec_())
    window.close_connection()