import CppHeaderParser
import struct

type_map = {
    "uint32_t": 'I',
    "uint16_t": 'H',
    "uint8_t": 'B',
    "int32_t": 'i',
    "int16_t": 'h',
    "int8_t": 'b',
    "float": 'f',
    "double": 'd'
}

byte_order = "<" # little endian

def _get_variable(variables, var_name):
    for var in variables:
        if var.get("name") == var_name:
            return var.get("default")
    return None

def _get_define(defines, field):
    for define in defines:
        if define.startswith(field):
            return define.replace(field, "").split("//")[0].strip()

def _get_enum(enums, enum_name):
    for enum in enums:
        if enum["name"] == enum_name:
            result = {}

            for value in enum["values"]:
                result[value["name"]] = value["value"]

            return result
    raise ValueError(f"Enum {enum_name} not found")

def parse_flash_interface(filepath):
    cpp_header = CppHeaderParser.CppHeader(filepath)

    # get usb magic value
    usb_magic_str = _get_define(cpp_header.defines, "FLASH_USB_MAGIC")
    if usb_magic_str is None:
        raise ValueError("FLASH_USB_MAGIC not found in defines")
    usb_magic_val = int(usb_magic_str, 16)

    # get enum values
    enum = _get_enum(cpp_header.enums, "flash_cmd_t")

    # get ack and nack values
    flash_ack_str = _get_variable(cpp_header.variables, "flash_ack")
    flash_nack_str = _get_variable(cpp_header.variables, "flash_nack")

    if flash_ack_str is None or flash_nack_str is None:
        raise ValueError("flash_ack or flash_nack not found in variables")

    # convert flash ack/nack to little-endian
    flash_ack_val = struct.pack('<I', int(flash_ack_str, 16))
    flash_nack_val = struct.pack('<I', int(flash_nack_str, 16))

    return usb_magic_val, enum, flash_ack_val, flash_nack_val

def parse_flash_header(filepath):
    cpp_header = CppHeaderParser.CppHeader(filepath)

    # get header magic value
    header_magic_str = _get_define(cpp_header.defines, "FLASH_HEADER_MAGIC")
    if header_magic_str is None:
        raise ValueError("FLASH_HEADER_MAGIC not found in defines")
    header_magic_val = int(header_magic_str, 16)

    # get flash header struct
    flash_header_struct = cpp_header.classes.get("flash_header_t")
    if not flash_header_struct: raise ValueError("Struct flash_header_t was not found.")

    fmt = byte_order
    fields = []

    for prop in flash_header_struct["properties"]["public"]:
        c_type = prop["type"]
        c_name = prop["name"]

        if c_type in type_map:
            fmt += type_map[c_type]
            fields.append(c_name)

        # handling nested structs
        elif c_type == "vector3f_t":
            fmt += "3f"
            fields.extend([f"{c_name}_x", f"{c_name}_y", f"{c_name}_z"])

    header_magic_format = fmt[fields.index("magic") + 1]
    header_magic_bytes = struct.pack(byte_order + header_magic_format, header_magic_val)
    header_magic_size = struct.calcsize(header_magic_format)

    return header_magic_size, header_magic_bytes, fmt, fields

def parse_flash_packet(filepath):
    cpp_header = CppHeaderParser.CppHeader(filepath)

    # get magic value
    packet_magic_str = _get_define(cpp_header.defines, "FLASH_PACKET_MAGIC")
    if packet_magic_str is None:
        raise ValueError("FLASH_PACKET_MAGIC not found in defines")
    packet_magic_val = int(packet_magic_str, 16)

    # get payload struct
    payload_struct = cpp_header.classes.get("flash_payload_t")
    if not payload_struct: raise ValueError("Struct flash_payload_t was not found.")

    # get packet struct
    packet_struct = cpp_header.classes.get("flash_packet_t")
    if not packet_struct: raise ValueError("Struct flash_packet_t was not found.")

    fmt = byte_order
    fields = []

    for prop in packet_struct["properties"]["public"]:
        c_type = prop["type"]
        c_name = prop["name"]

        if c_type == "flash_payload_t":
            for payload_prop in payload_struct["properties"]["public"]:
                payload_c_type = payload_prop["type"]
                payload_c_name = payload_prop["name"]

                if payload_c_type in type_map:
                    fmt += type_map[payload_c_type]
                    fields.append(payload_c_name)
                elif payload_c_type == "vector3f_t":
                    fmt += "3f"
                    fields.extend([f"{payload_c_name}_x", f"{payload_c_name}_y", f"{payload_c_name}_z"])

        elif c_type in type_map:
            fmt += type_map[c_type]
            fields.append(c_name)

        # handling nested structs
        elif c_type == "vector3f_t":
            fmt += "3f"
            fields.extend([f"{c_name}_x", f"{c_name}_y", f"{c_name}_z"])

    packet_magic_format = fmt[fields.index("magic") + 1]
    packet_magic_bytes = struct.pack(byte_order + packet_magic_format, packet_magic_val)
    packet_magic_size = struct.calcsize(packet_magic_format)

    return packet_magic_size, packet_magic_bytes, fmt, fields

if __name__ == "__main__":
    flash_log_path = "../../lib/flash_log/flash_log.h"

    # flash header
    header_magic_size, header_magic_bytes, header_fmt, header_fields = parse_flash_header(flash_log_path)
    print("FLASH HEADER")
    print("\tMagic Size:", header_magic_size)
    print("\tMagic Bytes:", header_magic_bytes)
    print("\tFormat:", header_fmt)
    print("\tFields:", header_fields)

    print()

    # flash packet
    packet_magic_size, packet_magic_bytes, packet_fmt, packet_fields = parse_flash_packet(flash_log_path)
    print("FLASH PACKET")
    print("\tMagic Size:", packet_magic_size)
    print("\tMagic Bytes:", packet_magic_bytes)
    print("\tFormat:", packet_fmt)
    print("\tFields:", packet_fields)
