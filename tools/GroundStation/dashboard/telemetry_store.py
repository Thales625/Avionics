from collections import defaultdict, deque

class TelemetryStore:
    def __init__(self, maxlen=200):
        self.maxlen = maxlen
        self.data = defaultdict(lambda: deque(maxlen=self.maxlen))

    def add(self, packet):
        for k, v in packet.items():
            self.data[k].append(v) 

    def get(self, key, index=None):
        if index is None:
            return self.data[key]

        try:
            return self.data[key][index]
        except:
            return None

    def clear(self, key=None):
        if key is None: # clear all
            for dq in self.data.values():
                dq.clear()
        else: # clear key
            if key in self.data:
                self.data[key].clear()
