from PyQt6.QtWidgets import QToolBar, QLabel, QComboBox, QPushButton, QMessageBox

class ConnToolbar(QToolBar):
    def __init__(self, telemetry_store, telemetry_link, title="Serial Connection"):
        super().__init__(title)

        self.title = title
        self.telemetry_link = telemetry_link
        self.telemetry_store = telemetry_store

        self.setMovable(False)

        # --- elements ---
        self.addWidget(QLabel("Port"))
        self.combo_ports = QComboBox()
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
        
        ports = self.telemetry_link.get_available_ports()

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
        if not self.telemetry_link.is_running:
            selected_port = self.combo_ports.currentText()
            success, msg = self.telemetry_link.connect(selected_port)
            
            if success:
                self.btn_connect.setText("Disconnect")
                self.btn_connect.setStyleSheet("background-color: #c62828; color: white; font-weight: bold;")
                self.combo_ports.setEnabled(False)
                self.btn_refresh.setEnabled(False)

                # clear stored data
                self.telemetry_store.clear()
            else:
                QMessageBox.critical(self, "Connection Error", msg)
        else:
            self.telemetry_link.disconnect()
            self.refresh_ports()

    def tick(self):
        if self.btn_connect.text() == "Disconnect" and not self.telemetry_link.is_running:
            self.telemetry_link.disconnect()
            self.refresh_ports()
            print("[Warning] USB connection was lost.")
            QMessageBox.critical(self, "Disconnect", "USB connection was lost.")
