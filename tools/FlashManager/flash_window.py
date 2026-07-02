from PyQt6.QtWidgets import QMainWindow, QDockWidget
from PyQt6.QtCore import Qt, QTimer

from link import Link
from store import Store
from conn_toolbar import ConnToolbar

from widgets.command import CommandWidget

class FlashWindow(QMainWindow):
    def __init__(self, title, store:Store):
        super().__init__()

        # --- link ---
        self.link = Link()

        # --- storage ---
        self.store = store

        # --- timer ---
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(50)

        # --- toolbar ---
        self.conn_toolbar = ConnToolbar(self.store, self.link)
        self.addToolBar(self.conn_toolbar)

        # --- command widget ---
        self.add_widget(CommandWidget("Command", self.link))

        # --- window ---
        self.setWindowTitle(title)
        self.resize(1024, 768)

    def add_widget(self, widget):
        dock = QDockWidget(widget.title, self)
        dock.setWidget(widget)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, dock)

    def update_ui(self):
        # update toolbars
        self.conn_toolbar.tick()

    def closeEvent(self, a0):
        # stop timer
        if self.timer.isActive(): self.timer.stop()

        # disconnect telemetry link
        self.link.disconnect()

        if a0 is not None: a0.accept()