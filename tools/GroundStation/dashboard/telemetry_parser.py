import CppHeaderParser
import struct

from logger import Logger

def _get_define(defines, field):
    for define in defines:
        if define.startswith(field):
            return define.replace(field, "").strip()

def parse_telecommand_header(filepath):
    header = CppHeaderParser.CppHeader(filepath)

    # get magic value
    magic_str = _get_define(header.defines, "TELECOMMAND_MAGIC")
    if magic_str is None:
        raise ValueError("TELECOMMAND_MAGIC not found in defines")
    magic_val = int(magic_str, 16)

    # get payload struct
    payload_struct_data = header.classes.get("telecommand_payload_t")
    if not payload_struct_data: raise ValueError("Struct telecommand_payload_t was not found.")

    # get packet struct
    packet_struct_data = header.classes.get("telecommand_packet_t")
    if not packet_struct_data: raise ValueError("Struct telecommand_packet_t was not found.")

    fmt = "<" # little endian
    fields = []

    # type mapping
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

    for prop in packet_struct_data["properties"]["public"]:
        c_type = prop["type"]
        c_name = prop["name"]

        if c_type == "telecommand_payload_t":
            for payload_prop in payload_struct_data["properties"]["public"]:
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

    magic_format = fmt[fields.index("magic") + 1]
    magic_bytes = struct.pack("<"+magic_format, magic_val)
    magic_size = struct.calcsize(magic_format)

    return magic_size, magic_bytes, fmt, fields

def parse_telemetry_header(filepath):
    header = CppHeaderParser.CppHeader(filepath)

    # get magic value
    magic_str = _get_define(header.defines, "TELEMETRY_MAGIC")
    if magic_str is None:
        raise ValueError("TELEMETRY_MAGIC not found in defines")
    magic_val = int(magic_str, 16)

    # get payload struct
    payload_struct_data = header.classes.get("lora_payload_t")
    if not payload_struct_data: raise ValueError("Struct lora_payload_t was not found.")

    # get packet struct
    packet_struct_data = header.classes.get("lora_packet_t")
    if not packet_struct_data: raise ValueError("Struct lora_packet_t was not found.")

    fmt = "<" # little endian
    fields = []

    # type mapping
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

    for prop in packet_struct_data["properties"]["public"]:
        c_type = prop["type"]
        c_name = prop["name"]

        if c_type == "lora_payload_t":
            for payload_prop in payload_struct_data["properties"]["public"]:
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

    magic_format = fmt[fields.index("magic") + 1]
    magic_bytes = struct.pack("<"+magic_format, magic_val)
    magic_size = struct.calcsize(magic_format)

    return magic_size, magic_bytes, fmt, fields

if __name__ == "__main__":
    tmtc_path = "../../../lib/tmtc/tmtc.h"

    # telecommand
    tc_magic_size, tc_magic_bytes, tc_fmt, tc_fields = parse_telecommand_header(tmtc_path)
    Logger.debug("TELECOMMAND")
    Logger.debug(tc_magic_size)
    Logger.debug(tc_magic_bytes)
    Logger.debug(tc_fmt)
    Logger.debug(tc_fields)

    # telemetry
    tm_magic_size, tm_magic_bytes, tm_fmt, tm_fields = parse_telemetry_header(tmtc_path)
    Logger.debug("TELEMETRY")
    Logger.debug(tm_magic_size)
    Logger.debug(tm_magic_bytes)
    Logger.debug(tm_fmt)
    Logger.debug(tm_fields)