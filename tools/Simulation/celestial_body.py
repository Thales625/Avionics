G = 6.674080324960551e-11

class CelestialBody:
    def __init__(self, mass:float, radius:float):
        self.radius = radius
        self.GM = G * mass

    def gravity(self, altitude):
        return -self.GM / (self.radius+altitude)**2

    def rho(self, altitude):
        return 1.225 * ((2.71828 ** (-altitude / 8500)) if altitude > 0 else 1)

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import numpy as np

    earth = CelestialBody(5.972365131085893e+24, 6371000)

    h_arr = np.linspace(0, 100000, 1000)

    plt.plot(h_arr, earth.gravity(h_arr))
    # plt.plot(h_arr, [earth.rho(h) for h in h_arr])
    plt.show()