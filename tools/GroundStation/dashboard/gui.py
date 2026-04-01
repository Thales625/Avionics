from PyQt6.QtWidgets import QApplication, QMainWindow, QDockWidget, QComboBox, QPushButton, QToolBar, QLabel, QWidget, QVBoxLayout, QMessageBox
from PyQt6.QtCore import Qt, QTimer
import pyqtgraph as pg

import sys
import queue
import serial.tools.list_ports

from telemetry import Telemetry

MAX_PACKET_HISTORY = 200

class GroundStationWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("URT - Ground Station")
        self.resize(1024, 768)

        # --- threading ---
        self.packet_queue = queue.Queue()
        self.backend = Telemetry(self.packet_queue)

        # --- history ---
        self.telemetry_history = []

        # --- setup components ---
        self._setup_toolbar()
        self._setup_docks()
        
        # --- timer ---
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(50) 

    def _setup_toolbar(self):
        toolbar = QToolBar("Serial connection")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        # combo box
        toolbar.addWidget(QLabel("Port"))
        self.combo_ports = QComboBox()
        toolbar.addWidget(self.combo_ports)

        self.btn_update_ports = QPushButton("Refresh")
        self.btn_update_ports.clicked.connect(self._update_ports)
        toolbar.addWidget(self.btn_update_ports)

        self.btn_connect = QPushButton("No ports")
        self.btn_connect.clicked.connect(self._toggle_conn)
        toolbar.addWidget(self.btn_connect)

        self._update_ports()

    def _update_ports(self):
        self.btn_update_ports.setEnabled(True)

        self.combo_ports.clear()
        ports = [port.device for port in serial.tools.list_ports.comports() if port.vid is not None]

        if ports:
            self.combo_ports.addItems(ports)
            self.combo_ports.setEnabled(True)

            self.btn_connect.setEnabled(True)
            self.btn_connect.setText("Connect")
            self.btn_connect.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold;")
        else:
            self.combo_ports.setEnabled(False)

            self.btn_connect.setEnabled(False)
            self.btn_connect.setText("No ports")
            self.btn_connect.setStyleSheet("background-color: #696969; color: white; font-weight: bold;")

    def _toggle_conn(self):
        if not self.backend.is_running: # connect
            selected_port = self.combo_ports.currentText()
            sucess, msg = self.backend.connect(selected_port)
            
            if sucess:
                self.btn_connect.setText("Disconnect")
                self.btn_connect.setStyleSheet("background-color: #c62828; color: white; font-weight: bold;")
                self.combo_ports.setEnabled(False)
                self.btn_update_ports.setEnabled(False)

                self.telemetry_history.clear()
            else:
                QMessageBox.critical(self, "Connection Error", msg)
        else: # disconnect
            self.backend.disconnect()
            self._update_ports()

    def _setup_docks(self):
        # --- status dock
        dock_status = QDockWidget("Flight status", self)
        dock_status.setFeatures(QDockWidget.DockWidgetFeature.DockWidgetMovable)
        
        content_status = QWidget()
        layout_status = QVBoxLayout()
        self.lbl_phase = QLabel("Phase: --")
        self.lbl_pressure = QLabel("Pressure: -- Pa")
        self.lbl_temp = QLabel("Temp: -- °C")
        
        for lbl in [self.lbl_phase, self.lbl_pressure, self.lbl_temp]:
            lbl.setStyleSheet("font-size: 16px; font-weight: bold;")
            layout_status.addWidget(lbl)
            
        content_status.setLayout(layout_status)
        dock_status.setWidget(content_status)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, dock_status)

        # --- graph dock
        dock_graph = QDockWidget("Graph: Accel Z (m/s^2)", self)
        self.graph = pg.PlotWidget()
        self.graph.setBackground('#121212')
        self.graph.showGrid(x=True, y=True, alpha=0.3)

        self.accel_z_curve = self.graph.plot([], [], pen=pg.mkPen(color='#00ff00', width=2))
        
        dock_graph.setWidget(self.graph)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, dock_graph)

    def update_ui(self):
        # disconnection detection
        if self.btn_connect.text() == "Disconnect" and not self.backend.is_running:
            self.backend.disconnect()
            self._update_ports()
            QMessageBox.warning(self, "Disconnected", "USB cable was disconnected!")

        # process packets
        packets_received = 0
        
        while not self.packet_queue.empty():
            packet = self.packet_queue.get()
            
            self.telemetry_history.append(packet)
            
            # update texts
            self.lbl_phase.setText(f"Phase: {packet['phase']}")
            self.lbl_pressure.setText(f"Pressure: {packet['pressure']:.1f} Pa")
            self.lbl_temp.setText(f"Temp: {packet['temp']:.1f} °C")
            
            packets_received += 1

        # update graph
        if packets_received > 0:
            if len(self.telemetry_history) > MAX_PACKET_HISTORY: self.telemetry_history = self.telemetry_history[-MAX_PACKET_HISTORY:]
                
            self.accel_z_curve.setData([p['ut'] for p in self.telemetry_history], [p['accel_z'] for p in self.telemetry_history])

    def closeEvent(self, event):
        self.backend.disconnect()
        event.accept()

def run():
    app = QApplication(sys.argv)
    app.setStyle("Fusion") 
    window = GroundStationWindow()
    window.show()
    sys.exit(app.exec())
