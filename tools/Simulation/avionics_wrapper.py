import ctypes
import flight_logic_bindings as _lib

class AvionicsSim:
    def __init__(self):
        self._core = _lib.flight_logic_t()

        # log callback
        def logger(msg):
            print(f"[{self._core.state.ut:04}] {msg}")
        self._cb = _lib.sim_log_callback_t(logger)
        _lib.flight_logic_set_sim_logger(self._cb)

        # time
        self.delay = 40. * 1e-3
        self._last_t = 0.0

    def init(self):
        _lib.flight_logic_init(ctypes.byref(self._core))

    def update(self, t):
        if t - self._last_t >= self.delay:
            _lib.flight_logic_update(ctypes.byref(self._core))
            self._last_t = t

    def set_sensors(self, ut, ax, ay, az, rx, ry, rz, press, temp):
        self._core.state.ut = int(ut)
        self._core.state.accel.x = float(ax)
        self._core.state.accel.y = float(ay)
        self._core.state.accel.z = float(az)
        self._core.state.ang_vel.x = float(rx)
        self._core.state.ang_vel.y = float(ry)
        self._core.state.ang_vel.z = float(rz)
        self._core.state.pressure = float(press)
        self._core.state.temperature = float(temp)

    # getters
    @property
    def phase(self): return self._core.state.phase

    @property
    def altitude(self): return self._core.altitude_baro

    @property
    def trigger_parachute(self): return self._core.trigger_parachute

    @property
    def trigger_shutdown(self): return self._core.trigger_shutdown
