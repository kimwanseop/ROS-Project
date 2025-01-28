import os 
import sys 
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import time 
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import *
from PyQt5 import uic

ui_file = "./UI/User.ui"
form_class = uic.loadUiType(ui_file)[0]

class Init_User_Server(QMainWindow, form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.WINDOW_TYPES = {
            'main_window': False,
            'map': False,
            'car' : False,
            'setting' : False 
        }
        self.OPEN_WINDOW = {
            'main_window': self.show_main_window,    
            'map':self.show_map_window,        
            'car':self.show_car_window,
        }
        self.HIDE_WINDOW = {
            'main_window': self.hide_main_window,
            'map':self.hide_map_window,
            'car':self.hide_car_window,        
        }
        self.initalize()
    
    def initalize(self):
        self.show_main_window()
        self.hide_map_window()
        self.hide_car_window()

    def show_main_window(self):
        self.start_widget.show()

    def hide_main_window(self):
        self.start_widget.hide()
    
    def show_map_window(self):
        self.map_widget.show()

    def hide_map_window(self):
        self.map_widget.hide()

    def show_car_window(self):
        self.car_widget.show()

    def hide_car_window(self):
        self.car_widget.hide()

