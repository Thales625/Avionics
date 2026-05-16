import CppHeaderParser
import struct

from logger import Logger

def _get_define(defines, field):
    for define in defines:
        if define.startswith(field):
            return define.replace(field, "").strip()

def parse_telemetry_header(filepath):
    header = CppHeaderParser.CppHeader(filepath)

    # get magic value
    magic_str = _get_define(header.defines, "TELEMETRY_MAGIC")
    if magic_str is None:
        raise ValueError("TELEMETRY_MAGIC not found in defines")
    magic_val = int(magic_str, 16)

    # get packet struct
    struct_data = header.classes.get("telemetry_packet_t")
    if not struct_data: raise ValueError("Struct telemetry_packet_t was not found.")

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

    for prop in struct_data["properties"]["public"]:
        c_type = prop["type"]
        c_name = prop["name"]

        if c_type in type_map:
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
    magic_size, magic_bytes, fmt, fields = parse_telemetry_header("../../../lib/tmtc/tmtc.h")

    Logger.debug(magic_size)
    Logger.debug(magic_bytes)
    Logger.debug(fmt)
    Logger.debug(fields)
