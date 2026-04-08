class TelecommandBus:
    def __init__(self, queue):
        self._queue = queue

    def send(self, telecommand):
        self._queue.put(telecommand)
