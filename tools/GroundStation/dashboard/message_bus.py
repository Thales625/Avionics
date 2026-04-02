class MessageBus:
    def __init__(self, queue):
        self._queue = queue

    def send(self, message):
        self._queue.put(message)
