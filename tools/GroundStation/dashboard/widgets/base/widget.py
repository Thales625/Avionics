from PyQt6.QtWidgets import QWidget

class Widget(QWidget):
    def __init__(self, title, *, persistent=False, interval=None):
        super().__init__()

        self.title = title
        self.persistent = persistent
        self.active = True

        self.interval = interval # ms

        self._last_update = 0

    def tick(self):
        raise NotImplementedError
