from PyQt6.QtWidgets import QVBoxLayout, QHBoxLayout, QSpinBox, QLineEdit, QLabel, QPushButton, QFrame
from PyQt6.QtGui import QIntValidator
from PyQt6.QtCore import Qt

from logger import Logger

from .base.widget import Widget

class TelecommandWidget(Widget):
    def __init__(self, title, telemetry_link, **kwargs):
        super().__init__(title, **kwargs)

        self.link = telemetry_link

        layout = QVBoxLayout()
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)

        buttons_layout = QHBoxLayout()

        # ARM btn
        self.btn_arm = QPushButton("ARM")
        self.btn_arm.setMinimumHeight(40)
        self.btn_arm.setStyleSheet("QPushButton { background-color: #006600; color: white; font-weight: bold; border-radius: 5px; } QPushButton:hover { background-color: #008800; border: 2px solid #00ff66; }")
        self.btn_arm.clicked.connect(self.cmd_arm)

        # DISARM btn
        self.btn_disarm = QPushButton("DISARM")
        self.btn_disarm.setMinimumHeight(40)
        self.btn_disarm.setStyleSheet("QPushButton { background-color: #660000; color: white; font-weight: bold; border-radius: 5px; } QPushButton:hover { background-color: #880000; border: 2px solid #ff4444; }")
        self.btn_disarm.clicked.connect(self.cmd_disarm)

        # EJECT btn
        self.btn_eject = QPushButton("EJECT")
        self.btn_eject.setMinimumHeight(40)
        self.btn_eject.setStyleSheet("QPushButton { background-color: #cc6600; color: white; font-weight: bold; border-radius: 5px; } QPushButton:hover { background-color: #ff8800; border: 2px solid #ffcc00; }")
        self.btn_eject.clicked.connect(self.cmd_eject)

        buttons_layout.addWidget(self.btn_arm)
        buttons_layout.addWidget(self.btn_disarm)
        buttons_layout.addWidget(self.btn_eject)

        layout.addLayout(buttons_layout)

        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setFrameShadow(QFrame.Shadow.Sunken)
        line.setStyleSheet("background-color: #333333; margin-top: 15px; margin-bottom: 15px;")
        layout.addWidget(line)

        # --- DEBUGGER ---

        # ID (int16)
        id_layout = QHBoxLayout()
        id_layout.addWidget(QLabel("ID:"))

        self.id_input = QSpinBox()
        self.id_input.setRange(-32768, 32767)
        self.id_input.setValue(0)
        id_layout.addWidget(self.id_input)
        layout.addLayout(id_layout)

        # Param (uint32)
        param_layout = QHBoxLayout()
        param_layout.addWidget(QLabel("Param:"))

        self.param_input = QLineEdit()
        self.param_input.setPlaceholderText("0 - 4294967295")
        self.param_input.setValidator(QIntValidator(0, 2147483647))
        param_layout.addWidget(self.param_input)
        layout.addLayout(param_layout)

        layout.addStretch()

        self.send_button = QPushButton("Send Command")
        self.send_button.setMinimumHeight(30)
        self.send_button.clicked.connect(self.send_custom_command)
        layout.addWidget(self.send_button)

        self.setLayout(layout)

    def cmd_arm(self):
        self.link.transmit(id=self.link.TC_ENUM["TC_ARM"], param=self.link.TC_MAGIC_BYTES)
        Logger.info("ARM send")

    def cmd_disarm(self):
        self.link.transmit(id=self.link.TC_ENUM["TC_DISARM"], param=self.link.TC_MAGIC_BYTES)
        Logger.info("DISARM send")

    def cmd_eject(self):
        self.link.transmit(id=self.link.TC_ENUM["TC_PARACHUTE_EJECT"], param=self.link.TC_MAGIC_BYTES)
        Logger.info("EJECT send")

    def send_custom_command(self):
        cmd_id = self.id_input.value()

        try:
            text = self.param_input.text()
            param = int(text) if text else 0

            if not (0 <= param <= 0xFFFFFFFF):
                raise ValueError(f"{param} is out of uint32 bounds")
        except ValueError as e:
            Logger.warning(f"Invalid param: {e}")
            return

        self.link.transmit(cmd_id, param)
        Logger.info(f"Custom Command: ID={cmd_id}, Param={param}")

    def tick(self):
        pass