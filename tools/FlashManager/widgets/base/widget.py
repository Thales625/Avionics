from PyQt6.QtWidgets import QWidget

class Widget(QWidget):
    def __init__(self, title, *, persistent=False):
        super().__init__()

        self.title = title
        self.persistent = persistent
        self.active = True

    def tick(self):
        raise NotImplementedError
