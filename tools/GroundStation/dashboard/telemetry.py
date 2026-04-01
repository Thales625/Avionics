import struct
import threading
import serial

class Telemetry:
    def __init__(self, data_queue):
        self.data_queue = data_queue
        self.serial_port = None
        self.is_running = False
        self.thread = None

        self.PACKET_FORMAT = "<IIB8fH"
        self.PACKET_SIZE = struct.calcsize(self.PACKET_FORMAT)
        self.MAGIC_BYTES = struct.pack("<I", 0xAABBCCDD)

    def connect(self, port, baudrate=115200):
        try:
            self.serial_port = serial.Serial()
            self.serial_port.port = port
            self.serial_port.baudrate = baudrate
            self.serial_port.timeout = 0.5

            self.serial_port.dtr = False
            self.serial_port.rts = False

            self.serial_port.open()

            self.serial_port.reset_input_buffer()

            self.is_running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            return True, "Connected"
        except Exception as e:
            return False, f"Failed to connect: {e}"

    def disconnect(self):
        self.is_running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

    def _read_loop(self):
        sync_buffer = b''
        
        while self.is_running and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting > 0:
                    byte = self.serial_port.read(1)
                    sync_buffer += byte

                    # limit sync_buffer to 4 bytes
                    if len(sync_buffer) > 4: sync_buffer = sync_buffer[-4:]
                        
                    if sync_buffer == self.MAGIC_BYTES:
                        rest_of_packet = self.serial_port.read(self.PACKET_SIZE - 4)
                        
                        if len(rest_of_packet) == self.PACKET_SIZE - 4:
                            # unpack binary data into python variables
                            unpacked_data = struct.unpack(self.PACKET_FORMAT, self.MAGIC_BYTES + rest_of_packet)
                            
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
                            
                            self.data_queue.put(packet)

                        sync_buffer = b''
            except Exception as e:
                print(f"Error in serial reading: {e}")
                break

        self.is_running = False
        if self.serial_port and self.serial_port.is_open: 
            self.serial_port.close()
