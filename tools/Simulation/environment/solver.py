import numpy as np

class RK4:
    def __init__(self, state, dSdt) -> None:
        self.S = state # np.copy(state)
        self.dSdt = dSdt
    
    def step(self, t, dt):
        k1 = self.dSdt(self.S, t)
        k2 = self.dSdt(self.S + 0.5*dt*k1, t + 0.5*dt)
        k3 = self.dSdt(self.S + 0.5*dt*k2, t + 0.5*dt)
        k4 = self.dSdt(self.S + dt*k3, t + dt)
        self.S += (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
        return self.S

if __name__ == "__main__":
    def dSdt(S, t):
        x, y, vx, vy, theta, omega = S
        return np.array([vx, vy, 0, -1.62, omega, 0.0])

    S0 = np.array([
        0., # position
        0.,

        3., # velocity
        4.,

        0.0, # angle

        0.1, # angular velocity
    ])

    solver = RK4(S0, dSdt)

    for i in range(10): print(solver.step(0, 0.1))