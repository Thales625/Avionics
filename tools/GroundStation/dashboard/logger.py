from __future__ import annotations

import time
from enum import Enum

class LogLevel(Enum):
    INFO = ("INFO", "\033[32m")   # green
    WARN = ("WARN", "\033[93m")   # yellow
    ERROR = ("ERROR", "\033[31m") # red
    DEBUG = ("DEBUG", "\033[90m") # gray

    @property
    def label(self) -> str:
        return self.value[0]

    @property
    def color(self) -> str:
        return self.value[1]


class Logger:
    RESET = "\033[0m"

    _start_time = time.monotonic()

    @staticmethod
    def _elapsed() -> str:
        elapsed = int(time.monotonic() - Logger._start_time)

        hours = elapsed // 3600
        minutes = (elapsed % 3600) // 60
        seconds = elapsed % 60

        return f"{hours:02}:{minutes:02}:{seconds:02}"

    @staticmethod
    def _log(level: LogLevel, *message: object) -> None:
        timestamp = Logger._elapsed()

        text = " ".join(str(m) for m in message)

        print(
            f"{level.color}"
            f"[{timestamp}] "
            f"[{level.label}] "
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