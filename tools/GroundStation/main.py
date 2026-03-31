import struct
from serial import Serial

# <  : Little Endian
# I  : uint32_t (Magic)
# I  : uint32_t (Uptime)
# B  : uint8_t  (Phase)
# 8f : 8 floats (3 Accel + 3 AngVel + 1 Press + 1 Temp)
# H  : uint16_t (Checksum)
PACKET_FORMAT = "<IIB8fH"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
MAGIC_BYTES = struct.pack("<I", 0xAABBCCDD)

if __name__ == "__main__":
    PORT = "/dev/ttyUSB0" 
    BAUD = 115200

    print(f"Listening port {PORT} at {BAUD} bps...")
    print(f"Expected packet size: {PACKET_SIZE} bytes")
    
    try:
        ser = Serial(PORT, BAUD)
    except Exception as e:
        print(f"Error opening serial port: {e}")
        exit(1)

    sync_buffer = b''

    while True:
        try:
            byte = ser.read(1)
            sync_buffer += byte

            # print(byte.hex(), end=" ", flush=True) # debug
            
            # limit sync_buffer to 4 bytes
            if len(sync_buffer) > 4: sync_buffer = sync_buffer[-4:]
                
            if sync_buffer == MAGIC_BYTES:
                rest_of_packet = ser.read(PACKET_SIZE - 4)
                
                if len(rest_of_packet) == PACKET_SIZE - 4:
                    # unpack binary data into python variables
                    unpacked_data = struct.unpack(PACKET_FORMAT, MAGIC_BYTES + rest_of_packet)
                    
                    magic = unpacked_data[0]
                    ut = unpacked_data[1]
                    phase = unpacked_data[2]
                    accel_x, accel_y, accel_z = unpacked_data[3:6]
                    gyro_x, gyro_y, gyro_z = unpacked_data[6:9]
                    press = unpacked_data[9]
                    temp = unpacked_data[10]
                    checksum = unpacked_data[11]

                    print(f"[{ut}ms] Phase: {phase} | Accel Z: {accel_z:.2f} m/s² | Pressure: {press:.1f} Pa | Temp: {temp} | Checksum: {checksum}")
                
                sync_buffer = b'' # reset sync buffer
                
        except KeyboardInterrupt:
            print("\nStopping telemetry.")
            break
        except Exception as e:
            print(f"Read error: {e}")
