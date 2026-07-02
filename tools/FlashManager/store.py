from collections import defaultdict

class Store:
    def __init__(self):
        self.data = defaultdict(list)
        self.cursor = None

    def add(self, packet):
        for k, v in packet.items():
            self.data[k].append(v)

    def load(self, packets):
        self.clear()

        for packet in packets:
            self.add(packet)

    def get(self, key, index=None):
        values = self.data[key]

        if index is None:
            if self.cursor is None:
                # return entire flight
                return values

            # return data up to index
            return values[:self.cursor + 1]

        # relative index
        if index < 0:
            if self.cursor is None:
                index = len(values) + index
            else:
                index = self.cursor + index

        # limit cursor
        if self.cursor is not None:
            index = min(index, self.cursor)

        # index bounds
        if 0 <= index < len(values):
            return values[index]

        return None

    def set_index(self, index):
        if len(self.data) == 0:
            self.cursor = None
            return

        self.cursor = max(0, min(index, len(self) - 1))

    def clear(self, key=None):
        if key is None: # clear all
            self.data.clear()
            self.cursor = None
        else: # clear key
            self.data.pop(key, None)

    def __len__(self):
        if not self.data: return 0

        return max(len(v) for v in self.data.values())