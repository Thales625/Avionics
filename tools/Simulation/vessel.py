import numpy as np

from solver import RK4

class Vessel:
    def __init__(self, dry_mass, celestial_body) -> None:
        self.state = np.array([
            0., # altitude
            0., # velocity
        ])

        self.thrust = 0.
        self.acceleration = 0.

        self.dry_mass = dry_mass

        self.engines = []

        # drag
        self.cd = 0.4
        self.radius = 0.1

        # sensors
        self.baro_delay = 275.0 * 1e-3
        self._baro = self.pressure(0)
        self._baro_t = 0.0

        self.acc_delay = 1.0 * 1e-3
        self._acc = celestial_body.gravity(0)
        self._acc_t = 0.0

        # solver
        def dSdt(S, t):
            h, v, = S

            mass = self.mass(t)

            if h < 0: # ground collision
                self.state[0] = 0 # altitude = 0
                self.state[1] = 0 # velocity = 0
                v = 0
                h = 0

            self.acceleration = (
                (
                    self.thrust +
                    -(1.57075*self.radius**2)*self.cd*celestial_body.rho(h)*v*abs(v)
                ) / mass + 
                celestial_body.gravity(h)
            )

            return np.array([
                v,                 # dh/dt
                self.acceleration, # dv/dt
            ])

        self.solver = RK4(self.state, dSdt)
    
    @property
    def altitude(self):
        return self.state[0]
    @altitude.setter
    def altitude(self, value):
        self.state[0] = value

    @property
    def velocity(self):
        return self.state[1]
    @velocity.setter
    def velocity(self, value):
        self.state[1] = value

    def mass(self, t):
        return self.dry_mass + sum([engine.mass(t) for engine in self.engines])

    def add_engine(self, engine):
        self.engines.append(engine)

    def update(self, dt, ut):
        # update thrust
        self.thrust = 0.
        for engine in self.engines:
            self.thrust += engine.thrust(ut)

        # step ivp
        self.solver.step(ut, dt)

        # log
        # print(f"T: {ut:.2f} | F: {self.thrust:.2f} | M: {self.mass(ut):.2f}")

    # sensors
    def baro(self, t):
        if t - self._baro_t >= self.baro_delay:
            self._baro = self.pressure(self.altitude) + np.random.normal(0, 0.5)
            self._baro_t = t
        return self._baro

    def acc(self, t):
        if t - self._acc_t >= self.acc_delay:
            self._acc = self.acceleration + np.random.normal(0, 0.1)
            self._acc_t = t
        return self._acc

    @classmethod
    def pressure(cls, altitude):
        return 101325.*(1.-altitude/44330.)**5.255

