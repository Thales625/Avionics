from PyQt6.QtWidgets import QApplication, QMainWindow, QDockWidget
from PyQt6.QtCore import Qt, QTimer

import sys
import queue
from time import monotonic

from telemetry_link import TelemetryLink, TelemetryLinkDebug
from telemetry_store import TelemetryStore
from conn_toolbar import ConnToolbar

from widgets.base.manager import WidgetManager

class MainWindow(QMainWindow):
    def __init__(self, title):
        super().__init__()

        # --- telemetry link ---
        self.packet_queue = queue.Queue()
        self.message_queue = queue.Queue()

        self.telemetry_link = TelemetryLink(self.packet_queue, self.message_queue)
        # self.telemetry_link = TelemetryLinkDebug(self.packet_queue, self.message_queue)

        # -- storage --
        self.store = TelemetryStore(100)

        # --- widget manager ----
        self.widget_manager = WidgetManager()
        
        # --- timer ---
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(50) 

        # --- toolbar ---
        self.conn_toolbar = ConnToolbar(self.store, self.telemetry_link)
        self.addToolBar(self.conn_toolbar)
        
        # -- window --
        self.setWindowTitle(title)
        self.resize(1024, 768)

    def add_widget(self, widget):
        self.widget_manager.register(widget)

        dock = QDockWidget(widget.title, self)
        dock.setWidget(widget)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, dock)

    def update_ui(self):
        # store packets
        while not self.packet_queue.empty():
            packet = self.packet_queue.get()
            self.store.add(packet)

        # update widgets
        self.widget_manager.tick(monotonic() * 1000)
        
        # update connection toolbar
        self.conn_toolbar.tick()

    def closeEvent(self, event):
        self.telemetry_link.disconnect()
        event.accept()

def init():
    app = QApplication(sys.argv)
    app.setStyle("Fusion") 
    return app
