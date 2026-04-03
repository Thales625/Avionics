import struct
import threading
import serial
import serial.tools.list_ports
from time import sleep

from random import uniform

class TelemetryLinkDebug:
    def __init__(self, packet_queue, message_queue):
        self.packet_queue = packet_queue
        self.message_queue = message_queue

        self.serial_port = None
        self.is_running = False
        self.thread = None

        self.PACKET_FORMAT = "<IIB8fH"
        self.PACKET_SIZE = struct.calcsize(self.PACKET_FORMAT)
        self.MAGIC_BYTES = struct.pack("<I", 0xAABBCCDD)

    @staticmethod
    def get_available_ports():
        return ["ttyUSB0", "ttyUSB1"]

    def connect(self, port, baudrate=115200):
        try:
            self.is_running = True
            self.thread = threading.Thread(target=self._loop, daemon=True)
            self.thread.start()
            return True, "Connected"
        except Exception as e:
            return False, f"Failed to connect: {e}"

    def disconnect(self):
        self.is_running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

    def _loop(self):
        dt = 0.2
        t = 0
        while self.is_running:
            # read packet from esp32
            sleep(dt)
            t += dt
            
            packet = {
                'ut': float(t)*1000,
                'phase': int(1),
                'accel_x': uniform(-1.0, 1.0),
                'accel_y': uniform(-1.0, 1.0),
                'accel_z': uniform(-1.0, 1.0),
                'gyro_x': uniform(-3.0, 3.0),
                'gyro_y': uniform(-3.0, 3.0),
                'gyro_z': uniform(-3.0, 3.0),
                'pressure': uniform(0, 1.2),
                'temp': uniform(25.0, 30.0)
            }
            
            self.packet_queue.put(packet)

            # send message packet to esp32
            while not self.message_queue.empty():
                message = self.message_queue.get()
                print(f"SENDING MESSAGE: {message}")

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

        self.PACKET_FORMAT = "<IIB8fH"
        self.PACKET_SIZE = struct.calcsize(self.PACKET_FORMAT)
        self.MAGIC_BYTES = struct.pack("<I", 0xAABBCCDD)

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

                # limit sync_buffer to 4 bytes
                if len(sync_buffer) > 4: sync_buffer = sync_buffer[-4:]
                    
                if sync_buffer == self.MAGIC_BYTES:
                    rest_of_packet = self.serial_port.read(self.PACKET_SIZE - 4)
                    
                    if len(rest_of_packet) == self.PACKET_SIZE - 4:
                        # unpack binary data into python variables
                        full_packet = self.MAGIC_BYTES + rest_of_packet
                        unpacked_data = struct.unpack(self.PACKET_FORMAT, full_packet)
                        payload_bytes = full_packet[:-2]
                        
                        packet = {
                            'ut': unpacked_data[1],
                            'phase': unpacked_data[2],
                            'accel_x': unpacked_data[3],
                            'accel_y': unpacked_data[4],
                            'accel_z': unpacked_data[5],
                            'gyro_x': unpacked_data[6],
                            'gyro_y': unpacked_data[7],
                            'gyro_z': unpacked_data[8],
                            'pressure': unpacked_data[9],
                            'temp': unpacked_data[10]
                        }

                        # valid packet
                        if unpacked_data[11]==self.crc16(payload_bytes):
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
