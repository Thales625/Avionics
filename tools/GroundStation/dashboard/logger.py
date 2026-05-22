from __future__ import annotations

import time
from enum import Enum

class LogLevel(Enum):
    INFO = ("I", "\033[32m")   # green
    WARN = ("W", "\033[93m")  # yellow
    ERROR = ("E", "\033[31m") # red
    DEBUG = ("D", "\033[90m") # gray

    @property
    def label(self) -> str:
        return self.value[0]

    @property
    def color(self) -> str:
        return self.value[1]


class Logger:
    RESET = "\033[0m"

    @staticmethod
    def _elapsed() -> str:
        return time.strftime("%H:%M:%S", time.localtime())

    @staticmethod
    def _log(level: LogLevel, *message: object) -> None:
        timestamp = Logger._elapsed()

        text = " ".join(str(m) for m in message)

        print(
            f"{level.color}"
            f"{level.label} "
            f"[{timestamp}] "
            f"{text}"
            f"{Logger.RESET}"
        )

    @staticmethod
    def info(*message: object) -> None:
        Logger._log(LogLevel.INFO, *message)

    @staticmethod
    def warning(*message: object) -> None:
        Logger._log(LogLevel.WARN, *message)

    @staticmethod
    def error(*message: object) -> None:
        Logger._log(LogLevel.ERROR, *message)

    @staticmethod
    def debug(*message: object) -> None:
        Logger._log(LogLevel.DEBUG, *message)

if __name__ == "__main__":
    Logger.info("Logger test")