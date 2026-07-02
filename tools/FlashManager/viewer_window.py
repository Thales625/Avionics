from PyQt6.QtWidgets import QMainWindow, QDockWidget
from PyQt6.QtCore import Qt, QTimer

from widgets.base.manager import WidgetManager
from playback_toolbar import PlaybackToolbar

class ViewerWindow(QMainWindow):
    def __init__(self, title, store):
        super().__init__()

        # --- storage ---
        self.store = store

        # --- widget manager ----
        self.widget_manager = WidgetManager()

        # --- playback toolbar ---
        self.playback_toolbar = PlaybackToolbar(self.store)
        self.addToolBar(Qt.ToolBarArea.BottomToolBarArea, self.playback_toolbar)

        # --- timer ---
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(50)

        # --- window ---
        self.setWindowTitle(title)
        self.resize(1024, 768)

    def add_widget(self, widget):
        self.widget_manager.register(widget)

        dock = QDockWidget(widget.title, self)
        dock.setWidget(widget)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, dock)

    def update_ui(self):
        # update widgets
        self.widget_manager.tick()

        # update playback toolbar
        self.playback_toolbar.tick()

    def closeEvent(self, a0):
        # stop timer
        if self.timer.isActive(): self.timer.stop()

        if a0 is not None: a0.accept()
