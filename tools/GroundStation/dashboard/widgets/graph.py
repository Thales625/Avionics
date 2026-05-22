from PyQt6.QtWidgets import QVBoxLayout
import pyqtgraph as pg

from .base.widget import Widget

class GraphWidget(Widget):
    def __init__(self, title, telemetry_store, x_signal, y_signals, width=2, min_y=None,  max_y=None, **kwargs):
        super().__init__(title, **kwargs)

        self.store = telemetry_store

        self.x_signal = x_signal
        self.y_signals = self.y_signals = [y_signals] if isinstance(y_signals, str) else y_signals

        self.min_y, self.max_y = min_y, max_y

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

        data_min = float("inf")
        data_max = float("-inf")
        has_data = False

        for sig, curve in self.curves.items():
            y_deque = self.store.get(sig)
            if len(y_deque) == 0: continue

            n = min(len(x_deque), len(y_deque))
            y_data = list(y_deque)[-n:]
            curve.setData(x_data[-n:], y_data)

            if self.min_y is not None and self.max_y is not None:
                data_min = min(data_min, min(y_data))
                data_max = max(data_max, max(y_data))
                has_data = True

        # auto scale
        if has_data and self.min_y is not None and self.max_y is not None:
            plot_min = min(self.min_y, data_min)
            plot_max = max(self.max_y, data_max)

            if data_max > self.max_y: plot_max += (plot_max - plot_min) * 0.05
            if data_min < self.min_y: plot_min -= (plot_max - plot_min) * 0.05

            self.graph.setYRange(plot_min, plot_max, padding=0)
