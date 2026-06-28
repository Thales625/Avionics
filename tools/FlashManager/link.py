import struct
import serial
import serial.tools.list_ports
from time import sleep
from pathlib import Path

from logger import Logger

from parser import parse_flash_header, parse_flash_packet, parse_flash_interface

class Link:
    def __init__(self):
        self.is_running = False
        self.serial_port = serial.Serial()
        self.serial_port.timeout = 3.0

        # get packets info
        base_dir = Path(__file__).resolve().parent
        flash_log_path = (base_dir / "../../lib/flash_log/flash_log.h").resolve()
        flash_interface_path = (base_dir / "../../lib/flash_log/flash_interface.h").resolve()

        # get header struct
        try:
            self.HEADER_MAGIC_SIZE, self.HEADER_MAGIC_BYTES, self.HEADER_FORMAT, self.HEADER_FIELDS = parse_flash_header(flash_log_path)
        except Exception as e:
            Logger.error(f"<Parser> Fatal error processing flash header: {e}")
            exit()

        # get packet struct
        try:
            self.PACKET_MAGIC_SIZE, self.PACKET_MAGIC_BYTES, self.PACKET_FORMAT, self.PACKET_FIELDS = parse_flash_packet(flash_log_path)
        except Exception as e:
            Logger.error(f"<Parser> Fatal error processing flash packet: {e}")
            exit()

        # get header size
        self.HEADER_SIZE = struct.calcsize(self.HEADER_FORMAT)

        # get packet size
        self.PACKET_SIZE = struct.calcsize(self.PACKET_FORMAT)

        # get usb magic
        self.USB_MAGIC, self.FLASH_CMD = parse_flash_interface(flash_interface_path)

    @staticmethod
    def get_available_ports():
        return [port.device for port in serial.tools.list_ports.comports() if port.vid is not None]

    def connect(self, port, baudrate=115200):
        try:
            self.serial_port.port = port
            self.serial_port.baudrate = baudrate

            self.serial_port.open()

            # fix linux bug
            self.serial_port.baudrate = 9600
            sleep(0.1)
            self.serial_port.baudrate = baudrate

            # flush buffer
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()

            self.is_running = True

            # /DEBUG
            sleep(1)
            Logger.debug("List Headers")
            self.cmd_list_headers()
            return True, "Connected"

            Logger.debug("Read Header")
            self.cmd_read_header(2)

            Logger.debug("Read Flight")
            self.cmd_read_flight(2)
            # \DEBUG

            return True, "Connected"
        except Exception as e:
            self.disconnect()
            return False, f"Failed to connect: {e}"

    def disconnect(self):
        self.is_running = False

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()


    def cmd_list_headers(self):
        if not self.is_running:
            return None

        cmd_id = self.FLASH_CMD["CMD_LIST_HEADERS"]
        cmd_param = 0
        cmd_bytes = struct.pack("<IIi", self.USB_MAGIC, cmd_id, cmd_param)

        self.serial_port.write(cmd_bytes)

        # read response
        while True:
            response = self.serial_port.read(self.HEADER_SIZE)

            # break on timeout
            if len(response) != self.HEADER_SIZE:
                break

            unpacked_header = struct.unpack(self.HEADER_FORMAT, response)
            header = dict(zip(self.HEADER_FIELDS, unpacked_header))

            print(header)

    def cmd_read_header(self, flight_number):
        if not self.is_running:
            return None

        cmd_id = self.FLASH_CMD["CMD_READ_HEADER"]
        cmd_param = flight_number

        cmd_bytes = struct.pack("<IIi", self.USB_MAGIC, cmd_id, cmd_param)
        self.serial_port.write(cmd_bytes)
        self.serial_port.flush()

        # read response
        header = None
        while True:
            response = self.serial_port.read(self.HEADER_SIZE)

            # break on timeout
            if len(response) != self.HEADER_SIZE:
                break

            unpacked_header = struct.unpack(self.HEADER_FORMAT, response)
            header = dict(zip(self.HEADER_FIELDS, unpacked_header))

            print(header)

        return header

    def cmd_read_flight(self, flight_number):
        if not self.is_running:
            return None

        cmd_id = self.FLASH_CMD["CMD_READ_FLIGHT"]
        cmd_param = flight_number

        packet = struct.pack("<IIi", self.USB_MAGIC, cmd_id, cmd_param)
        self.serial_port.write(packet)
        self.serial_port.flush()

        # read response
        while True:
            response = self.serial_port.read(self.PACKET_SIZE)

            Logger.debug(f"Response: {response}")
            print(len(response), self.PACKET_SIZE)

            # break on timeout
            if len(response) != self.PACKET_SIZE:
                break

            unpacked_data = struct.unpack(self.PACKET_FORMAT, response)
            packet = dict(zip(self.PACKET_FIELDS, unpacked_data))

            print(packet)

        return packet