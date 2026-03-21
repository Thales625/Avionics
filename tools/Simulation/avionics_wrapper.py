import ctypes
import flight_logic_bindings as _lib

class AvionicsSim:
    def __init__(self):
        self._core = _lib.flight_logic_t()

        # log callback
        def logger(msg):
            print(f"[{self._core.sensor_data.ut:04}] {msg}")
        self._cb = _lib.sim_log_callback_t(logger)
        _lib.flight_logic_set_sim_logger(self._cb)

    def init(self):
        _lib.flight_logic_init(ctypes.byref(self._core))

    def update(self):
        _lib.flight_logic_update(ctypes.byref(self._core))

    def set_sensors(self, ut, ax, ay, az, rx, ry, rz, press, temp):
        self._core.sensor_data.ut = int(ut)
        self._core.sensor_data.accel.x = float(ax)
        self._core.sensor_data.accel.y = float(ay)
        self._core.sensor_data.accel.z = float(az)
        self._core.sensor_data.rot.x = float(rx)
        self._core.sensor_data.rot.y = float(ry)
        self._core.sensor_data.rot.z = float(rz)
        self._core.sensor_data.pressure = float(press)
        self._core.sensor_data.temperature = float(temp)

    # getters
    @property
    def state(self): return self._core.state
    
    @property
    def altitude(self): return self._core.altitude_baro
    
    @property
    def trigger_parachute(self): return self._core.trigger_parachute
    
    @property
    def trigger_shutdown(self): return self._core.trigger_shutdown
