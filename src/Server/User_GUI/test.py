from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QDialog, QLabel, QApplication
from PyQt5.QtGui import QPixmap, QMouseEvent
from PyQt5.QtCore import Qt
import sys

class ClickableLabel(QLabel):
    """ 클릭 이벤트가 감지되는 QLabel 클래스 """
    def __init__(self, parent=None):
        super().__init__(parent)

    def mousePressEvent(self, event: QMouseEvent):
        """ 마우스 클릭 이벤트 처리 """
        if event.button() == Qt.LeftButton:  # 왼쪽 버튼 클릭 시
            x, y = event.x(), event.y()  # 클릭한 위치 가져오기
            self.setText(f"클릭 위치: ({x}, {y})")  # QLabel에 좌표 표시
            print(f"클릭한 위치: ({x}, {y})")  # 터미널에도 출력

class PositionDialog(QDialog):
    """ QDialog 기반의 위치 선택 창 """
    def __init__(self):
        super().__init__()
        uic.loadUi('./UI/select_position.ui', self)  # UI 파일 로드

        # 🔥 QLabel(map)이 정상적으로 로드되었는지 확인 (오류 방지)
        self.map = self.findChild(QLabel, "map")  # QLabel 찾기
        if self.map is None:
            print("❌ QLabel(map)을 찾을 수 없습니다!")
            return

        # 기존 QLabel을 ClickableLabel로 변경
        self.layout().removeWidget(self.map)  # 기존 QLabel 제거
        self.map.deleteLater()  # 메모리에서 제거
        self.map = ClickableLabel(self)  # 새로운 QLabel 생성
        self.layout().addWidget(self.map)  # 다시 추가

        self.btn_call.clicked.connect(self.renting_car)
        self.btn_cancel.clicked.connect(self.close_dialog)

        # QLabel에 이미지 추가
        self.pixmap = QPixmap("your_image.png")  # 이미지 경로 설정
        self.map.setPixmap(self.pixmap)

    def renting_car(self):
        """ 차 호출 기능 (예제용) """
        print("렌트카 호출 버튼 클릭됨")

    def close_dialog(self):
        """ 다이얼로그 닫기 """
        self.close()

def main():
    app = QApplication(sys.argv)
    window = PositionDialog()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
