if __name__ == "__main__":
    import sys

    from main_window import MainWindow, init

    from widgets.graph import GraphWidget
    from widgets.status import StatusWidget

    app = init()

    window = MainWindow("URT - Ground Station")

    # --- add widgets --- 
    window.add_widget(StatusWidget("Flight status", window.store, {
        "ut": lambda ut: f"UT: {ut:.2f}",
        "pressure": lambda press: f"Press: {press:.2f}"
    }))
    
    # accel graph
    window.add_widget(GraphWidget("Acceleration", window.store,
        "ut", ["accel_x", "accel_y", "accel_z"]
    ))

    # gyro graph
    window.add_widget(GraphWidget("Gyro", window.store,
        "ut", ["gyro_x", "gyro_y", "gyro_z"]
    ))

    # display window
    window.show()

    sys.exit(app.exec())
