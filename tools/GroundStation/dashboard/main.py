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
            window.store,
            {
                "phase": lambda val: f"Phase: {val}",
                "ut": lambda val: f"ut: {val/1000:.2f} s",
                "pressure": lambda val: f"Press: {val:.0f} Pa"
            },
            interval=200
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
