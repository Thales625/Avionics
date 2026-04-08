from PyQt6.QtWidgets import QVBoxLayout, QLabel
from PyQt6.QtCore import Qt

from .base.widget import Widget

class StatusWidget(Widget):
    def __init__(self, title, telemetry_link, telemetry_store, label_generators, **kwargs):
        super().__init__(title, **kwargs)

        self.link = telemetry_link
        self.store = telemetry_store
        self.labels = []
        layout = QVBoxLayout()

        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)

        for signal_key, formatter_func in label_generators.items():
            lbl = QLabel()
            lbl.setStyleSheet("""
                QLabel {
                    font-size: 16px; 
                    font-weight: bold;
                    color: white;
                    background: transparent;
                }
            """)

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

        self.wdt_label = QLabel()
        self.wdt_label.setText("WDT: -- s")
        layout.addWidget(self.wdt_label)

        self.setLayout(layout)

    def tick(self):
        # update wdt
        time_without_signal = self._last_update - self.link.wdt_last_packet
        self.wdt_label.setText(f"WDT: {time_without_signal:.1f} s")
        if time_without_signal >= self.link.WDT_TIMEOUT:
            self.wdt_label.setStyleSheet("font-size: 16px; font-weight: bold; color: #c62828;")
        else:
            self.wdt_label.setStyleSheet("font-size: 16px; font-weight: bold; color: white;")

        # update labels
        for lbl in self.labels:
            latest_value = self.store.get(lbl.signal_key, -1)

            if latest_value:
                lbl.setText(lbl.formatter_func(latest_value))
