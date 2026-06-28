from PyQt6.QtWidgets import QApplication, QMainWindow, QDockWidget
from PyQt6.QtCore import Qt, QTimer

import sys

from link import Link
from store import Store
from conn_toolbar import ConnToolbar

from widgets.base.manager import WidgetManager

class MainWindow(QMainWindow):
    def __init__(self, title):
        super().__init__()

        # --- link ---
        self.link = Link()

        # --- storage ---
        self.store = Store()

        # --- widget manager ----
        self.widget_manager = WidgetManager()

        # --- timer ---
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(50)

        # --- toolbar ---
        self.conn_toolbar = ConnToolbar(self.store, self.link)
        self.addToolBar(self.conn_toolbar)

        # --- window ---
        self.setWindowTitle(title)
        self.resize(1024, 768)

    def add_widget(self, widget):
        self.widget_manager.register(widget)

        dock = QDockWidget(widget.title, self)
        dock.setWidget(widget)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, dock)

    def update_ui(self):
        # store packets
        # self.store.add(packet)

        # update widgets
        self.widget_manager.tick()

        # update connection toolbar
        self.conn_toolbar.tick()

    def closeEvent(self, a0):
        # stop timer
        if self.timer.isActive(): self.timer.stop()

        # disconnect telemetry link
        self.link.disconnect()

        if a0 is not None: a0.accept()

def init():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    return app
