import struct
import threading
import serial
import serial.tools.list_ports
import queue
from time import sleep, monotonic
from pathlib import Path

from logger import Logger

from telemetry_parser import parse_telecommand_header, parse_telemetry_header

class TelemetryLink:
    def __init__(self, telemetry_queue, telecommand_queue):
        self.telemetry_queue = telemetry_queue
        self.telecommand_queue = telecommand_queue

        self.is_running = False
        self.serial_port = serial.Serial()
        self.thread: threading.Thread | None = None

        self.wdt_last_packet = monotonic()
        self.WDT_TIMEOUT = 2.0

        self.HAS_RSSI = True

        # get packets info
        base_dir = Path(__file__).resolve().parent
        header_path = (base_dir / "../../../lib/tmtc/tmtc.h").resolve()

        # get TELECOMMAND struct
        try:
            self.TC_MAGIC_SIZE, self.TC_MAGIC_BYTES, self.TC_PACKET_FORMAT, self.TC_PACKET_FIELDS = parse_telecommand_header(header_path)
        except Exception as e:
            Logger.error(f"<Parser> Fatal error processing telecommand header: {e}")
            exit()

        # get TELEMETRY struct
        try:
            self.TM_MAGIC_SIZE, self.TM_MAGIC_BYTES, self.TM_PACKET_FORMAT, self.TM_PACKET_FIELDS = parse_telemetry_header(header_path)
        except Exception as e:
            Logger.error(f"<Parser> Fatal error processing telemetry header: {e}")
            exit()

        # get packet size
        self.TM_PACKET_SIZE = struct.calcsize(self.TM_PACKET_FORMAT)
        self.TC_PACKET_SIZE = struct.calcsize(self.TC_PACKET_FORMAT)

    @staticmethod
    def crc16(data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    @staticmethod
    def get_available_ports():
        return [port.device for port in serial.tools.list_ports.comports() if port.vid is not None]

    def connect(self, port, baudrate=115200):
        try:
            self.serial_port.port = port
            self.serial_port.baudrate = baudrate
            self.serial_port.timeout = 0.5

            self.serial_port.dtr = False
            self.serial_port.rts = False

            self.serial_port.open()

            # fix linux bug
            self.serial_port.baudrate = 9600
            sleep(0.05)
            self.serial_port.baudrate = baudrate

            # reset esp
            self.serial_port.dtr = False
            self.serial_port.rts = True
            sleep(0.1)
            self.serial_port.dtr = False
            self.serial_port.rts = False

            # wait for boot
            sleep(0.4)

            # flush buffer
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()

            self.is_running = True
            self.thread = threading.Thread(target=self._loop, daemon=True)
            self.thread.start()

            return True, "Connected"
        except Exception as e:
            self.disconnect()
            return False, f"Failed to connect: {e}"

    def disconnect(self):
        self.is_running = False

        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

    def transmit(self, id, param):
        if not self.is_running: return False

        packet = {
            "id": id,
            "param": param
        }

        self.telecommand_queue.put(packet)

        return True

    def _loop(self):
        sync_buffer = b''

        while self.is_running and self.serial_port.is_open:
            # TELECOMMAND
            try:
                while True:
                    tc = self.telecommand_queue.get_nowait()

                    values = []

                    for field in self.TC_PACKET_FIELDS:
                        if field == "magic":
                            values.append(int.from_bytes(self.TC_MAGIC_BYTES, byteorder="little"))
                        elif field == "checksum":
                            values.append(0)
                        else:
                            value = tc.get(field, 0)
                            if isinstance(value, float):
                                value = float(value)
                            elif isinstance(value, bytes):
                                value = int.from_bytes(value, byteorder="little", signed=True)
                                # value = int.from_bytes(value, byteorder="big", signed=True)
                            values.append(value)

                    packet = struct.pack(self.TC_PACKET_FORMAT, *values)
                    payload_bytes = packet[self.TC_MAGIC_SIZE : -2]
                    calculated_crc = self.crc16(payload_bytes)

                    values[self.TC_PACKET_FIELDS.index("checksum")] = calculated_crc
                    packet = struct.pack(self.TC_PACKET_FORMAT, *values)

                    self.serial_port.write(packet)

            except queue.Empty:
                pass

            # TELEMETRY
            try:
                byte = self.serial_port.read(1)
                if not byte: continue

                sync_buffer += byte

                # Logger.debug(f"Sync buffer: {sync_buffer}")

                # limit sync_buffer to MAGIC size
                if len(sync_buffer) > self.TM_MAGIC_SIZE: sync_buffer = sync_buffer[-self.TM_MAGIC_SIZE:]

                if sync_buffer == self.TM_MAGIC_BYTES:
                    # read payload
                    data_len = self.TM_PACKET_SIZE - self.TM_MAGIC_SIZE + (1 if self.HAS_RSSI else 0)
                    rest_of_packet = self.serial_port.read(data_len)
                    if len(rest_of_packet) == data_len:
                        full_packet = self.TM_MAGIC_BYTES + (rest_of_packet[:-1] if self.HAS_RSSI else rest_of_packet)

                        # unpack binary packet
                        unpacked_data = struct.unpack(self.TM_PACKET_FORMAT, full_packet)

                        # data to dict
                        packet = dict(zip(self.TM_PACKET_FIELDS, unpacked_data))

                        # valid packet
                        # if packet["checksum"]==self.crc16(full_packet[4:-2]):
                        if packet["checksum"]==self.crc16(rest_of_packet[:-3] if self.HAS_RSSI else rest_of_packet[:-2]):
                            # remove validation data
                            del packet["magic"]
                            del packet["checksum"]

                            if self.HAS_RSSI:
                                packet["rssi"] = rest_of_packet[-1] - 256

                            # update WDT
                            now = monotonic()
                            # Logger.debug(f"HZ = {1/(now - self.wdt_last_packet)}")
                            self.wdt_last_packet = now

                            packet["ut"] /= 1000 # ms to s

                            self.telemetry_queue.put(packet)
                        else:
                            Logger.warning("CHECKSUM UNMATCH!")

                    sync_buffer = b''

            except Exception as e:
                Logger.error(f"serial reading failed: {e}")
                break

        self.is_running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
