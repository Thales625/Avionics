import numpy as np
import matplotlib.pyplot as plt

class Engine:
    def __init__(self, thrust_curve_file:str) -> None:
        self._time_arr = []
        self._thrust_arr = []
        self._mass_arr = []

        if thrust_curve_file.endswith(".rse"):
            pass
        elif thrust_curve_file.endswith(".txt"):
            # t, f
            data = []

             # read file
            try:
                with open(thrust_curve_file, 'r') as f:
                    for line in f:
                        line = line.strip()

                        if not line: continue

                        part = line.split()

                        if len(part) < 2: continue

                        data.append((float(part[0]), float(part[1]), 0.))
                        # data.append((float(part[0]), float(part[1]), 0.5))
            except Exception as e:
                print(f"Error reading file: {e}")
                exit()
        else:
            data = [ # t, f, m, cg
                ("0.","0.","11.3","35."),
                ("0.04","0.229","11.2948","35."),
                ("0.12","0.658","11.2544","35."),
                ("0.211","1.144","11.1611","35."),
                ("0.291","1.831","11.0257","35."),
                ("0.385","2.86","10.7748","35."),
                ("0.447","3.833","10.5387","35."),
                ("0.505","5.001","10.2472","35."),
                ("0.567","3.89","9.93361","35."),
                ("0.615","3.146","9.74146","35."),
                ("0.665","2.66","9.5763","35."),
                ("0.735","2.203","9.38263","35."),
                ("0.815","2.088","9.18732","35."),
                ("0.93","1.98","8.92116","35."),
                ("4.589","1.96","0.719051","35."),
                ("4.729","1.888","0.412551","35."),
                ("4.815","1.602","0.24179","35."),
                ("4.873","1.259","0.147381","35."),
                ("4.969","0.658","0.0426774","35."),
                ("5.083","0.","0.","35."),
            ]

        for d in data:
            self._time_arr.append(float(d[0]))
            self._thrust_arr.append(float(d[1]))
            self._mass_arr.append(float(d[2]))

        self.max_t = max(self._time_arr)

    def thrust(self, t):
        if t > self.max_t: return 0
        return np.interp(t, self._time_arr, self._thrust_arr)

    def mass(self, t):
        return np.interp(t if t < self.max_t else self.max_t, self._time_arr, self._mass_arr)

    def plot(self):
        plt.plot(self._time_arr, self._thrust_arr)
        plt.show()

if __name__ == "__main__":
    eng = Engine("./thrust.txt")
    print(eng._mass_arr)
    eng.plot()
