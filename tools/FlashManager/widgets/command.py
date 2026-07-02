from PyQt6.QtWidgets import QVBoxLayout, QHBoxLayout, QPushButton, QFrame
from PyQt6.QtCore import Qt

from logger import Logger

from .base.widget import Widget

UINT8_RANGE = (0, 255)
INT32_RANGE = (-(1<<31), (1<<31)-1)

class CommandWidget(Widget):
    def __init__(self, title, link, **kwargs):
        super().__init__(title, **kwargs)

        self.link = link

        layout = QVBoxLayout()
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)

        buttons_layout = QHBoxLayout()

        # list flights btn
        self.btn_list_flights = QPushButton("List flights")
        self.btn_list_flights.setMinimumHeight(40)
        self.btn_list_flights.setStyleSheet("QPushButton { background-color: #448560; color: white; font-weight: bold; border-radius: 5px; } QPushButton:hover { background-color: #880000; border: 2px solid #ff4444; }")
        self.btn_list_flights.clicked.connect(self.cmd_list_flights)

        # clear flights btn
        self.btn_clear_flights = QPushButton("Clear flights")
        self.btn_clear_flights.setMinimumHeight(40)
        self.btn_clear_flights.setStyleSheet("QPushButton { background-color: #660000; color: white; font-weight: bold; border-radius: 5px; } QPushButton:hover { background-color: #008800; border: 2px solid #00ff66; }")
        self.btn_clear_flights.clicked.connect(self.cmd_clear_flights)

        buttons_layout.addWidget(self.btn_clear_flights)
        buttons_layout.addWidget(self.btn_list_flights)

        layout.addLayout(buttons_layout)

        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setFrameShadow(QFrame.Shadow.Sunken)
        line.setStyleSheet("background-color: #333333; margin-top: 15px; margin-bottom: 15px;")
        layout.addWidget(line)

        self.setLayout(layout)

    def cmd_clear_flights(self):
        Logger.info("Clear flights cmd")
        self.link.cmd_clear_flights()

    def cmd_list_flights(self):
        Logger.info("List flights cmd")
        self.link.cmd_list_headers()

    def tick(self):
        pass