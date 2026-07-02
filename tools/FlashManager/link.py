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

        # get flash interface info
        self.USB_MAGIC, self.FLASH_CMD, self.FLASH_ACK, self.FLASH_NACK = parse_flash_interface(flash_interface_path)

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

            return True, "Connected"
        except Exception as e:
            self.disconnect()
            return False, f"Failed to connect: {e}"

    def disconnect(self):
        self.is_running = False

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()


    def transmit_cmd(self, id, param):
        cmd_bytes = struct.pack("<IIi", self.USB_MAGIC, id, param)
        self.serial_port.write(cmd_bytes)


    def cmd_clear_flights(self) -> None:
        self.transmit_cmd(self.FLASH_CMD["CMD_CLEAR_FLIGHTS"], self.USB_MAGIC)

    def cmd_list_headers(self) -> list[dict]:
        if not self.is_running:
            return []

        self.transmit_cmd(self.FLASH_CMD["CMD_LIST_HEADERS"], 0)

        headers = []

        # read response
        while True:
            response = self.serial_port.read(self.HEADER_SIZE)

            # break on timeout
            if len(response) != self.HEADER_SIZE:
                break

            unpacked_header = struct.unpack(self.HEADER_FORMAT, response)
            header = dict(zip(self.HEADER_FIELDS, unpacked_header))

            headers.append(header)

            print(header)

        return headers

    def cmd_read_header(self, flight_number) -> dict | None:
        if not self.is_running:
            return None

        self.transmit_cmd(self.FLASH_CMD["CMD_READ_HEADER"], flight_number)

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

    def cmd_read_flight(self, flight_number) -> list[dict]:
        if not self.is_running:
            return []

        self.transmit_cmd(self.FLASH_CMD["CMD_READ_FLIGHT"], flight_number)

        packets = []
        sync_buffer = b''

        # read response
        while True:
            rx_byte = self.serial_port.read(1)
            if not rx_byte: continue

            sync_buffer += rx_byte

            # limit sync_buffer to 32bit size
            if len(sync_buffer) > self.PACKET_MAGIC_SIZE: sync_buffer = sync_buffer[-self.PACKET_MAGIC_SIZE:]

            Logger.debug("Sync Buffer:", sync_buffer)

            if sync_buffer == self.FLASH_NACK:
                Logger.error("NACK received")
                return packets

            elif sync_buffer == self.FLASH_ACK:
                Logger.info("ACK received")
                break

            elif sync_buffer == self.PACKET_MAGIC_BYTES:
                data_len = self.PACKET_SIZE - self.PACKET_MAGIC_SIZE
                rest_of_packet = self.serial_port.read(data_len)
                if len(rest_of_packet) == data_len:
                    full_packet = self.PACKET_MAGIC_BYTES + rest_of_packet

                    unpacked_data = struct.unpack(self.PACKET_FORMAT, full_packet)
                    packet = dict(zip(self.PACKET_FIELDS, unpacked_data))

                    packets.append(packet)

                    Logger.info(packet)

        return packets