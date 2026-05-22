from PyQt6.QtWidgets import QVBoxLayout
import pyqtgraph as pg

from .base.widget import Widget

import rasterio
from math import radians, log, tan, pi

class GeoMapper:
    EARTH_RADIUS = 6378137.0

    def __init__(self, tif_path):
        self.dataset = rasterio.open(tif_path)
        self.img = self.dataset.read([1, 2, 3]).transpose(1, 2, 0)

        # print(self.dataset.crs)
        # print(self.dataset.bounds)

    def _latlon_to_pixel(self, lat, lon):
        return self.dataset.index(lon, lat)

    def latlon_to_pixel(self, lat, lon):
        x, y = self.latlon_to_webmercator(lat, lon)
        row, col = self.dataset.index(x, y)
        if not (0 <= col < self.dataset.width and 0 <= row < self.dataset.height):
            return None, None
        return col, row

    @classmethod
    def latlon_to_webmercator(cls, lat_deg, lon_deg):
        lat_rad = radians(lat_deg)
        lon_rad = radians(lon_deg)

        x = cls.EARTH_RADIUS * lon_rad
        y = cls.EARTH_RADIUS * log(tan(pi / 4 + lat_rad / 2))

        return x, y

    @staticmethod
    def nmea_to_degrees(value_nmea):
        sign = -1 if value_nmea < 0 else 1
        value_abs = abs(value_nmea)

        nmea_float = value_abs / 100000.0

        degrees = int(nmea_float // 100)
        minutes = nmea_float - (degrees * 100)

        decimal_degrees = degrees + (minutes / 60.0)

        return sign * decimal_degrees

class GpsWidget(Widget):
    def __init__(self, title, telemetry_store, lat_signal, lon_signal, tif_path, **kwargs):
        super().__init__(title, **kwargs)

        self.store = telemetry_store

        self.lat_signal = lat_signal
        self.lon_signal = lon_signal

        self.mapper = GeoMapper(tif_path)

        self.graph = pg.PlotWidget()
        self.graph.setTitle(title)
        self.graph.setBackground("#121212")

        self.graph.getAxis("bottom").setStyle(showValues=False)
        self.graph.getAxis("left").setStyle(showValues=False)

        self.graph.setAspectLocked(True)
        self.graph.invertY(True)

        self.img_item = pg.ImageItem(self.mapper.img, axisOrder="row-major")
        self.graph.addItem(self.img_item)

        pen = pg.mkPen(color="#ff0000", width=2, style=pg.QtCore.Qt.PenStyle.SolidLine)
        self.trajectory_curve = self.graph.plot([], [], pen=pen, name="Trajectory")

        self.current_pos = pg.ScatterPlotItem(
            size=12,
            pen=pg.mkPen(None),
            brush=pg.mkBrush("#00ff00")
        )
        self.graph.addItem(self.current_pos)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.graph)

    def tick(self):
        lat_deque = self.store.get(self.lat_signal)
        lon_deque = self.store.get(self.lon_signal)

        if not lat_deque or not lon_deque: return

        n = min(len(lat_deque), len(lon_deque))
        lat_data = list(lat_deque)[-n:]
        lon_data = list(lon_deque)[-n:]

        # lat_data = list(lat_deque)[:n]
        # lon_data = list(lon_deque)[:n]

        x_coords = []
        y_coords = []

        for lat_nmea, lon_nmea in zip(lat_data, lon_data):
            lat_deg = GeoMapper.nmea_to_degrees(lat_nmea)
            lon_deg = GeoMapper.nmea_to_degrees(lon_nmea)

            px, py = self.mapper.latlon_to_pixel(lat_deg, lon_deg)

            if px is not None and py is not None:
                x_coords.append(px)
                y_coords.append(py)

        if x_coords:
            self.trajectory_curve.setData(x_coords, y_coords)

            self.current_pos.setData([x_coords[-1]], [y_coords[-1]])