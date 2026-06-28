if __name__ == "__main__":
    import sys

    from main_window import MainWindow, init

    from widgets.graph import GraphWidget
    from widgets.status import StatusWidget
    from widgets.gps import GpsWidget

    app = init()

    window = MainWindow("URT - Flash Manager")

    # --- add widgets ---

    # status dock
    window.add_widget(
        StatusWidget(
            "Flight status",
            window.store,
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
    window.add_widget(
        GraphWidget(
            "Acceleration",
            window.store,
            "ut", "accel_mag",
            min_y=0, max_y=5
        )
    )

    # gyro graph
    window.add_widget(
        GraphWidget(
            "Gyro",
            window.store,
            "ut", "ang_vel_mag",
            min_y=0, max_y=180
        )
    )

    # altitude graph
    window.add_widget(
        GraphWidget(
            "Altitude",
            window.store,
            "ut", "altitude",
            min_y=-10, max_y=150
        )
    )

    # gps widget
    window.add_widget(
        GpsWidget(
            "GPS",
            window.store,
            "lat_nmea", "lon_nmea",
            "assets/maps/pelotas.tiff",
        )
    )

    # display window
    window.show()

    sys.exit(app.exec())
