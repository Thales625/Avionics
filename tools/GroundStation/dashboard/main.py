if __name__ == "__main__":
    import sys

    from main_window import MainWindow, init

    from widgets.graph import GraphWidget
    from widgets.status import StatusWidget

    app = init()

    window = MainWindow("URT - Ground Station")

    # --- add widgets --- 

    # status dock
    window.add_widget(
        StatusWidget(
            "Flight status",
            window.telemetry_link,
            window.store,
            {
                "rssi": lambda val: f"RSSI: {val} dBm",
                "ut": lambda val: f"ut: {val:.1f} s",
                "phase": lambda val: f"Phase: {val}",
                "pressure": lambda val: f"Press: {val:.0f} Pa",
                "temperature": lambda val: f"Temp: {val:.0f} °C"
            },
            # interval=0.2 # 200ms
        )
    )
    
    # accel graph
    window.add_widget(
        GraphWidget(
            "Acceleration",
            window.store,
            "ut", ["accel_x", "accel_y", "accel_z"]
        )
    )

    # gyro graph
    window.add_widget(
        GraphWidget(
            "Gyro",
            window.store,
            "ut", ["ang_vel_x", "ang_vel_y", "ang_vel_z"]
        )
    )

    # display window
    window.show()

    sys.exit(app.exec())
