from PyQt6.QtWidgets import QVBoxLayout
import pyqtgraph as pg

from .base.widget import Widget

class GraphWidget(Widget):
    def __init__(self, title, telemetry_store, x_signal, y_signals, width=2, **kwargs):
        super().__init__(title, **kwargs)

        self.store = telemetry_store

        self.x_signal = x_signal
        self.y_signals = self.y_signals = [y_signals] if isinstance(y_signals, str) else y_signals

        self.curve_width = width
        self.curves = {}

        self.graph = pg.PlotWidget()
        self.graph.setTitle(title)
        self.graph.setBackground('#121212')
        self.graph.showGrid(x=True, y=True, alpha=0.3)
        legend = self.graph.addLegend(offset=(1, 1))
        legend.setBrush('#1e1e1e')
        
        layout = QVBoxLayout(self)
        layout.addWidget(self.graph)

        # create curves
        hues = len(self.y_signals)

        for i, sig in enumerate(self.y_signals):
            pen = pg.mkPen(pg.intColor(i, hues=hues), width=self.curve_width)
            curve = self.graph.plot([], [], pen=pen, name=sig)
            self.curves[sig] = curve
    
    def tick(self):
        x_deque = self.store.get(self.x_signal)
        if len(x_deque) == 0: return

        x_data = list(x_deque)

        for sig, curve in self.curves.items():
            y_deque = self.store.get(sig)
            if len(y_deque) == 0: continue

            n = min(len(x_deque), len(y_deque))
            curve.setData(x_data[-n:], list(y_deque)[-n:])
