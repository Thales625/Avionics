if __name__ == "__main__":
    import sys
    from pathlib import Path

    from main_window import MainWindow, init

    from widgets.graph import GraphWidget
    from widgets.status import StatusWidget
    from widgets.gps import GpsWidget
    from widgets.telecommand import TelecommandWidget
    from telemetry_parser import parse_flight_logic_header

    app = init()

    window = MainWindow("URT - Ground Station")

    # get flight phase enum
    base_dir = Path(__file__).resolve().parent
    header_path = (base_dir / "../../../lib/flight_logic/flight_logic.h").resolve()
    flight_phase_enum = parse_flight_logic_header(header_path)
    phase_by_value = {v: k.replace("PHASE_", "").replace("_", " ").lower().capitalize() for k, v in flight_phase_enum.items()}

    # --- add widgets ---

    # status dock
    window.add_widget(
        StatusWidget(
            "Flight status",
            window.telemetry_link,
            window.store,
            {
                "rssi": lambda val: f"RSSI: {val} dBm",
                "ut": lambda val: f"Time: {val:.1f} s",
                "phase": lambda val: f"Phase: {phase_by_value[val]}",
                "pressure": lambda val: f"Press: {val:.0f} Pa",
                "temperature": lambda val: f"Temp: {val:.0f} °C",
                "satellites": lambda val: f"Satellites: {val}",
                "v_bat": lambda val: f"Battery: {val:.2f} V"
            },
            interval=0.2 # 200ms
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
            interval=0.5 # 500ms
        )
    )

    # telecommand widget
    window.add_widget(
        TelecommandWidget(
            "Telecommand",
            window.telemetry_link
        )
    )

    # display window
    window.show()

    sys.exit(app.exec())
