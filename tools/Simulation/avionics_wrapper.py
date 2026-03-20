import ctypes
from os import path

class Vector3f_t(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float)
    ]

class SensorData_t(ctypes.Structure):
    _fields_ = [
        ("ut", ctypes.c_uint32),
        ("accel", Vector3f_t),
        ("rot", Vector3f_t),
        ("pressure", ctypes.c_float),
        ("temperature", ctypes.c_float)
    ]

class FlightLogic_t(ctypes.Structure):
    _fields_ = [
        ("state", ctypes.c_int),

        ("sensor_data", SensorData_t),

        ("ut_0", ctypes.c_uint32),
        ("pressure_0", ctypes.c_float),

        ("altitude_baro", ctypes.c_float),
        ("max_altitude_baro", ctypes.c_float),
        ("prev_altitude_baro", ctypes.c_float),

        ("parachute_ejection_count", ctypes.c_uint32),
        ("descent_stable_count", ctypes.c_uint32),

        ("trigger_parachute", ctypes.c_bool),
        ("trigger_shutdown", ctypes.c_bool)
    ]

class AvionicsSim:
    def __init__(self, lib_path="./libavionics.so"):
        abs_path = path.abspath(lib_path)
        self._lib = ctypes.CDLL(abs_path)
        
        self._lib.flight_logic_init.argtypes = [ctypes.POINTER(FlightLogic_t)]
        self._lib.flight_logic_init.restype = None
        
        self._lib.flight_logic_update.argtypes = [ctypes.POINTER(FlightLogic_t)]
        self._lib.flight_logic_update.restype = None
        
        self._core = FlightLogic_t()

        # log callback
        CALLBACK_TYPE = ctypes.CFUNCTYPE(None, ctypes.c_char_p)
        def logger(msg):
            print(f"[{self._core.sensor_data.ut:04}]: {msg.decode()}")
        self._cb = CALLBACK_TYPE(logger)
        self._lib.flight_logic_set_sim_logger.argtypes = [CALLBACK_TYPE]
        self._lib.flight_logic_set_sim_logger(self._cb)

    def init(self):
        self._lib.flight_logic_init(ctypes.byref(self._core))

    def update(self):
        self._lib.flight_logic_update(ctypes.byref(self._core))

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
