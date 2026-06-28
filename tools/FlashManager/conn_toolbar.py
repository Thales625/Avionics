from PyQt6.QtWidgets import QToolBar, QLabel, QComboBox, QPushButton, QMessageBox

from logger import Logger

class ConnToolbar(QToolBar):
    def __init__(self, store, link, title="Serial Connection"):
        super().__init__(title)

        self.title = title
        self.link = link
        self.store = store

        self.setMovable(False)

        # --- elements ---
        self.addWidget(QLabel("Port"))
        self.combo_ports = QComboBox()
        self.combo_ports.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContents)
        self.addWidget(self.combo_ports)

        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.addWidget(self.btn_refresh)

        self.btn_connect = QPushButton("No ports")
        self.btn_connect.clicked.connect(self.toggle_connection)
        self.addWidget(self.btn_connect)

        self.refresh_ports()

    def refresh_ports(self):
        self.btn_refresh.setEnabled(True)
        self.combo_ports.clear()

        ports = self.link.get_available_ports()

        if ports: # ports found
            self.combo_ports.addItems(ports)
            self.combo_ports.setEnabled(True)
            self.btn_connect.setEnabled(True)
            self.btn_connect.setText("Connect")
            self.btn_connect.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold;")
        else: # no ports found
            self.combo_ports.setEnabled(False)
            self.btn_connect.setEnabled(False)
            self.btn_connect.setText("No ports")
            self.btn_connect.setStyleSheet("background-color: #696969; color: white; font-weight: bold;")

    def toggle_connection(self):
        if not self.link.is_running:
            selected_port = self.combo_ports.currentText()
            success, msg = self.link.connect(selected_port)

            if success:
                self.btn_connect.setText("Disconnect")
                self.btn_connect.setStyleSheet("background-color: #c62828; color: white; font-weight: bold;")
                self.combo_ports.setEnabled(False)
                self.btn_refresh.setEnabled(False)

                # clear stored data
                self.store.clear()
            else:
                QMessageBox.critical(self, "Connection Error", msg)
        else:
            self.link.disconnect()
            self.refresh_ports()

    def tick(self):
        if self.btn_connect.text() == "Disconnect" and not self.link.is_running:
            self.link.disconnect()
            self.refresh_ports()
            Logger.warning("USB connection was lost.")
            QMessageBox.critical(self, "Disconnect", "USB connection was lost.")
