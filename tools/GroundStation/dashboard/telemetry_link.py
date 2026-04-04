import struct
import threading
import serial
import serial.tools.list_ports
from time import sleep

from telemetry_parser import parse_telemetry_header

class TelemetryLink:
    """
    TODO:
        - _loop needs to convert messages to packet and send to the esp32
    """
    def __init__(self, packet_queue, message_queue):
        self.packet_queue = packet_queue
        self.message_queue = message_queue

        self.is_running = False
        self.serial_port = None
        self.thread = None

        # get packet info
        self.MAGIC_SIZE, self.MAGIC_BYTES, self.PACKET_FORMAT, self.PACKET_FIELDS = parse_telemetry_header("../../../lib/tmtc/tmtc.h")
        if not self.PACKET_FORMAT: raise RuntimeError("Parser error")

        # get packet size
        self.PACKET_SIZE = struct.calcsize(self.PACKET_FORMAT)

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
            self.serial_port = serial.Serial()
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
            sleep(0.2)

            # flush buffer
            self.serial_port.reset_input_buffer()

            self.is_running = True
            self.thread = threading.Thread(target=self._loop, daemon=True)
            self.thread.start()

            return True, "Connected"
        except Exception as e:
            self.is_running = False
            return False, f"Failed to connect: {e}"

    def disconnect(self):
        self.is_running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

    def _loop(self):
        sync_buffer = b''
        
        while self.is_running and self.serial_port.is_open:
            try:
                byte = self.serial_port.read(1)
                if not byte: continue

                sync_buffer += byte

                # limit sync_buffer to MAGIC size
                if len(sync_buffer) > self.MAGIC_SIZE: sync_buffer = sync_buffer[-4:]
                    
                if sync_buffer == self.MAGIC_BYTES:
                    # read payload
                    rest_of_packet = self.serial_port.read(self.PACKET_SIZE - self.MAGIC_SIZE)
                    if len(rest_of_packet) == self.PACKET_SIZE - self.MAGIC_SIZE:
                        full_packet = self.MAGIC_BYTES + rest_of_packet

                        # unpack binary packet
                        unpacked_data = struct.unpack(self.PACKET_FORMAT, full_packet)

                        # data to dict
                        packet = dict(zip(self.PACKET_FIELDS, unpacked_data))

                        # valid packet (without checksum field)
                        if packet["checksum"]==self.crc16(full_packet[:-2]):
                            # remove validation data
                            del packet["magic"]
                            del packet["checksum"]

                            self.packet_queue.put(packet)
                        else:
                            print("CHECKSUM UNMATCH!")

                    sync_buffer = b''

            except Exception as e:
                print(f"Error in serial reading: {e}")
                break

        self.is_running = False
        if self.serial_port and self.serial_port.is_open: 
            self.serial_port.close()
