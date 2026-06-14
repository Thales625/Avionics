import ctypes

import bindings.flight_logic_bindings as _lib

class AvionicsSim:
    def __init__(self):
        object.__setattr__(self, "_core", _lib.flight_logic_t())

        # log callback
        def logger(msg):
            print(f"[{self._core.state.ut:04}] {msg}")
        object.__setattr__(self, "_cb", _lib.sim_log_callback_t(logger))
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

    def __getattr__(self, name):
        return getattr(self._core, name)

    def __setattr__(self, name:str, value):
        if name.startswith('-'):
            object.__setattr__(self, name, value)
        else:
            setattr(self._core, name, value)

    def list_attribs(self):
        attrs = {}

        def recurse(obj, prefix=""):
            if not hasattr(type(obj), "_fields_"):
                return

            for name, field_type in obj._fields_:
                full_name = f"{prefix}.{name}" if prefix else name

                if hasattr(field_type, "_fields_"):
                    recurse(getattr(obj, name), full_name)
                else:
                    attrs[full_name] = field_type

        recurse(self._core)

        return attrs

if __name__ == "__main__":
    avionics = AvionicsSim()

    print("---Avionics Attribs---")
    for attrib_name, attrib_type in avionics.list_attribs().items():
        print(f"\t{attrib_name}: <{attrib_type.__name__}>")
    print("----------------------")