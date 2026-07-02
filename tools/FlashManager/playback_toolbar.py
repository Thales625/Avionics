from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QToolBar, QPushButton, QLabel, QSlider

class PlaybackToolbar(QToolBar):
    def __init__(self, store, title="Playback"):
        super().__init__(title)

        self.store = store

        self.setMovable(False)

        # --- controls ---
        self.btn_prev = QPushButton("◀")
        self.btn_prev.clicked.connect(self.prev_frame)
        self.addWidget(self.btn_prev)

        self.btn_next = QPushButton("▶")
        self.btn_next.clicked.connect(self.next_frame)
        self.addWidget(self.btn_next)

        self.addSeparator()

        self.lbl_frame = QLabel("0 / 0")
        self.addWidget(self.lbl_frame)

        self.addSeparator()

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setMinimumWidth(400)
        self.slider.setTracking(True)
        self.slider.valueChanged.connect(self.slider_changed)
        self.addWidget(self.slider)

        self.tick()

    def slider_changed(self, value):
        self.store.set_index(value)

    def prev_frame(self):
        self.store.set_index(self.store.cursor - 1)

    def next_frame(self):
        self.store.set_index(self.store.cursor + 1)

    def tick(self):
        total = len(self.store)

        if total == 0:
            self.slider.setEnabled(False)
            self.btn_prev.setEnabled(False)
            self.btn_next.setEnabled(False)
            self.lbl_frame.setText("0 / 0")
            return

        if self.store.cursor is None:
            self.store.set_index(0)

        self.slider.blockSignals(True)

        self.slider.setEnabled(True)
        self.slider.setRange(0, total - 1)
        if self.store.cursor is not None:
            self.slider.setValue(self.store.cursor)

        self.slider.blockSignals(False)

        if self.store.cursor is not None:
            self.btn_prev.setEnabled(self.store.cursor > 0)
            self.btn_next.setEnabled(self.store.cursor < total - 1)

            self.lbl_frame.setText(f"{self.store.cursor + 1} / {total}")