import sys
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QWidget, QTableView, QPushButton, QHBoxLayout, QLineEdit, QLabel
)
from PyQt5.QtCore import QAbstractTableModel, Qt, QVariant, QSortFilterProxyModel
from PyQt5.QtWidgets import QHeaderView


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
        return QVariant()


class CustomerInfoApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("QTableView와 검색 기능을 활용한 고객 정보 입력 창")
        self.setGeometry(100, 100, 800, 600)

        # 테이블 데이터 및 헤더 정의
        self.table_data = [
            ["John Doe", "123-456-7890", "john@example.com"],
            ["Jane Smith", "987-654-3210", "jane@example.com"],
            ["Emily Davis", "555-555-5555", "emily@example.com"],
            ["Michael Brown", "111-222-3333", "michael@example.com"],
        ]
        self.headers = ["Name", "Phone Number", "Email"]

        # 메인 위젯 및 레이아웃
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        # 검색 필드 추가
        self.add_search_bar()

        # QTableView 생성
        self.table_model = CustomerTableModel(self.table_data, self.headers)
        self.proxy_model = QSortFilterProxyModel()
        self.proxy_model.setSourceModel(self.table_model)
        self.proxy_model.setFilterKeyColumn(-1)  # 모든 열을 필터링 대상으로 설정
        self.proxy_model.setFilterCaseSensitivity(Qt.CaseInsensitive)  # 대소문자 구분 안 함

        self.table_view = QTableView()
        self.table_view.setModel(self.proxy_model)

        # 테이블 헤더 크기 조정
        self.setup_table_headers()

        self.layout.addWidget(self.table_view)

        # 하단 버튼 및 입력 필드
        self.create_bottom_controls()

    def add_search_bar(self):
        """검색 필드를 추가"""
        search_layout = QHBoxLayout()

        self.search_input = QLineEdit()
        self.search_input.setPlaceholderText("Search...")
        self.search_input.textChanged.connect(self.apply_filter)

        search_layout.addWidget(QLabel("Search:"))
        search_layout.addWidget(self.search_input)
        self.layout.addLayout(search_layout)

    def apply_filter(self):
        """검색 필터 적용"""
        search_text = self.search_input.text()
        self.proxy_model.setFilterFixedString(search_text)

    def setup_table_headers(self):
        """테이블 헤더 크기 조정"""
        header = self.table_view.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)  # 열을 테이블 너비에 맞게 확장
        self.table_view.verticalHeader().setVisible(False)  # 행 번호 숨기기(선택 사항)

    def create_bottom_controls(self):
        """하단 컨트롤: 입력 필드 및 버튼"""
        control_layout = QHBoxLayout()

        # 입력 필드
        self.name_input = QLineEdit()
        self.name_input.setPlaceholderText("Name")
        self.phone_input = QLineEdit()
        self.phone_input.setPlaceholderText("Phone Number")
        self.email_input = QLineEdit()
        self.email_input.setPlaceholderText("Email")

        # 버튼
        add_button = QPushButton("Add Customer")
        add_button.clicked.connect(self.add_customer)

        delete_button = QPushButton("Delete Selected")
        delete_button.clicked.connect(self.delete_selected_customer)

        # 컨트롤 추가
        control_layout.addWidget(self.name_input)
        control_layout.addWidget(self.phone_input)
        control_layout.addWidget(self.email_input)
        control_layout.addWidget(add_button)
        control_layout.addWidget(delete_button)

        self.layout.addLayout(control_layout)

    def add_customer(self):
        """새 고객 추가"""
        name = self.name_input.text()
        phone = self.phone_input.text()
        email = self.email_input.text()

        if name and phone and email:  # 필드가 모두 입력되었는지 확인
            self.table_model.data.append([name, phone, email])
            self.table_model.layoutChanged.emit()  # 모델 갱신
            self.name_input.clear()
            self.phone_input.clear()
            self.email_input.clear()

    def delete_selected_customer(self):
        """선택된 고객 삭제"""
        selected_indexes = self.table_view.selectionModel().selectedRows()
        for index in sorted(selected_indexes, reverse=True):
            del self.table_model.data[index.row()]
        self.table_model.layoutChanged.emit()  # 모델 갱신
        self.proxy_model = QSortFilterProxyModel()
        self.proxy_model.setSourceModel(self.table_model)
        self.proxy_model.setFilterKeyColumn(-1)  # 모든 열을 필터링 대상으로 설정
        self.proxy_model.setFilterCaseSensitivity(Qt.CaseInsensitive)  # 대소문자 구분 안 함
        self.table_view.setModel(self.proxy_model)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CustomerInfoApp()
    window.show()
    sys.exit(app.exec_())
