from PyQt6.QtWidgets import QVBoxLayout, QLabel

from .base.widget import Widget

class StatusWidget(Widget):
    def __init__(self, title, telemetry_store, label_generators, **kwargs):
        super().__init__(title, **kwargs)

        self.store = telemetry_store

        self.labels = []

        layout = QVBoxLayout()

        for signal_key, formatter_func in label_generators.items():
            lbl = QLabel()
            lbl.setStyleSheet("font-size: 16px; font-weight: bold;")

            # default value
            try:
                lbl.setText(formatter_func(0)) 
            except Exception:
                lbl.setText(f"{signal_key}: --")

            lbl.signal_key = signal_key
            lbl.formatter_func = formatter_func

            layout.addWidget(lbl)
            self.labels.append(lbl)

        layout.addStretch()
        self.setLayout(layout)

    def tick(self):
        for lbl in self.labels:
            latest_value = self.store.get(lbl.signal_key, -1)

            if latest_value:
                lbl.setText(lbl.formatter_func(latest_value))
