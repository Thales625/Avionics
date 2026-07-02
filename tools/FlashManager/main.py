if __name__ == "__main__":
    import sys

    from PyQt6.QtWidgets import QApplication

    from flash_window import FlashWindow
    from viewer_window import ViewerWindow

    from store import Store

    from widgets.graph import GraphWidget
    from widgets.status import StatusWidget
    from widgets.gps import GpsWidget

    # init app
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    store = Store()

    flash_window = FlashWindow("URT - Flash Manager", store)
    viewer_window = ViewerWindow("URT - Flight Viewer", store)

    # --- add widgets to viewer window ---

    # status dock
    viewer_window.add_widget(
        StatusWidget(
            "Flight status",
            viewer_window.store,
            {
                "ut": lambda val: f"Time: {val:.1f} s",
                "phase": lambda val: f"Phase: {val}",
                "pressure": lambda val: f"Press: {val:.0f} Pa",
                "temperature": lambda val: f"Temp: {val:.0f} °C",
                "satellites": lambda val: f"Satellites: {val}",
                "v_bat": lambda val: f"Battery: {val:.2f} V"
            },
        )
    )

    # accel graph
    viewer_window.add_widget(
        GraphWidget(
            "Acceleration",
            viewer_window.store,
            "ut", ["accel_x", "accel_y", "accel_z"],
        )
    )

    # gyro graph
    viewer_window.add_widget(
        GraphWidget(
            "Gyro",
            viewer_window.store,
            "ut", ["ang_vel_x", "ang_vel_y", "ang_vel_z"],
        )
    )

    # baro graph
    viewer_window.add_widget(
        GraphWidget(
            "Pressure",
            viewer_window.store,
            "ut", ["pressure", "temperature"],
        )
    )

    # battery level graph
    viewer_window.add_widget(
        GraphWidget(
            "Battery Voltage",
            viewer_window.store,
            "ut", "v_bat",
        )
    )

    # flight phase graph
    viewer_window.add_widget(
        GraphWidget(
            "Flight Phase",
            viewer_window.store,
            "ut", "phase",
        )
    )

    # gps widget
    viewer_window.add_widget(
        GpsWidget(
            "GPS",
            viewer_window.store,
            "lat_nmea", "lon_nmea",
            "assets/maps/pelotas.tiff",
        )
    )

    # display windows
    viewer_window.show()
    flash_window.show()

    sys.exit(app.exec())
