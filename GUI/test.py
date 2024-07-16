import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget, QWidget, QVBoxLayout, QLabel, QFormLayout, QLineEdit, QPushButton, QListWidget, QHBoxLayout, QStackedWidget

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("관리자용 GUI")
        self.setGeometry(100, 100, 800, 600)
        
        main_layout = QHBoxLayout()
        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)
        
        # 사이드 바 생성
        self.side_bar = QListWidget()
        self.side_bar.addItem("로그인 & 회원가입")
        self.side_bar.addItem("재고관리")
        self.side_bar.addItem("입고관리")
        self.side_bar.addItem("출고관리")
        self.side_bar.addItem("관제 시스템")
        
        # 중앙 내용
        self.content_area = QStackedWidget()
        
        # 각 페이지 생성
        self.login_signup_page = self.create_login_signup_page()
        self.inventory_page = self.create_inventory_page()
        self.inbound_page = self.create_inbound_page()
        self.outbound_page = self.create_outbound_page()
        self.control_page = self.create_control_page()
        
        self.content_area.addWidget(self.login_signup_page)
        self.content_area.addWidget(self.inventory_page)
        self.content_area.addWidget(self.inbound_page)
        self.content_area.addWidget(self.outbound_page)
        self.content_area.addWidget(self.control_page)
        
        # 사이드 바 클릭 시 페이지 변경
        self.side_bar.currentRowChanged.connect(self.display_page)
        
        main_layout.addWidget(self.side_bar)
        main_layout.addWidget(self.content_area)
        
    def create_login_signup_page(self):
        page = QWidget()
        layout = QVBoxLayout()

        form_layout = QFormLayout()
        self.username_input = QLineEdit()
        self.password_input = QLineEdit()
        self.password_input.setEchoMode(QLineEdit.Password)
        form_layout.addRow("Username:", self.username_input)
        form_layout.addRow("Password:", self.password_input)
        
        self.login_button = QPushButton("Login")
        self.signup_button = QPushButton("Sign Up")
        
        layout.addLayout(form_layout)
        layout.addWidget(self.login_button)
        layout.addWidget(self.signup_button)
        
        page.setLayout(layout)
        return page
        
    def create_inventory_page(self):
        page = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(QLabel("재고관리 기능"))
        page.setLayout(layout)
        return page
    
    def create_inbound_page(self):
        page = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(QLabel("입고관리 기능"))
        page.setLayout(layout)
        return page

    def create_outbound_page(self):
        page = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(QLabel("출고관리 기능"))
        page.setLayout(layout)
        return page

    def create_control_page(self):
        page = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(QLabel("관제 시스템 기능"))
        page.setLayout(layout)
        return page

    def display_page(self, index):
        self.content_area.setCurrentIndex(index)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
