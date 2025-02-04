import time 
import threading
from PyQt5.QtCore import QThread, pyqtSignal

class Thread(QThread):
    data = pyqtSignal()

    def __init__(self, sec=0.1, parent=None):
        super().__init__()
        self.main = parent
        self.running = True
        self.sec = sec

    def run(self):
        while self.running:
            self.data.emit()
            time.sleep(self.sec)

    def stop(self):
        self.running = False


class CarThread(threading.Thread):
    def __init__(self, target):
        super().__init__()
        self.target_function = target  
        self.running = True

    def run(self):
        if self.running: 
            self.target_function() 

    def stop(self):
        self.running = False 


