#!/usr/bin/env python3
"""Read-only RG2 Modbus RTU diagnostics through the UR tool serial bridge.

Requires exclusive serial ownership: Tool I/O configured for User, 1M/8E1,
24 V, with no other gripper driver or OnRobot tool polling running.
Never writes a gripper register or changes any configuration.
Register addresses and unit ID follow the pinned OnRobot_ROS2_Driver/RG.hpp.
"""

import argparse
import json
import socket
import struct
import time


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


def frame(payload):
    return payload + struct.pack('<H', crc16(payload))


def signed16(value):
    return value if value < 32768 else value - 65536


def receive(sock, count, deadline):
    data = bytearray()
    while len(data) < count:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('Incomplete serial response')
        sock.settimeout(remaining)
        chunk = sock.recv(count - len(data))
        if not chunk:
            raise ConnectionError('Bridge closed during response')
        data.extend(chunk)
    return bytes(data)


def tool_configuration(host):
    """Read actual tool settings from UR's read-only primary interface.

    Layout: UR Primary/Secondary Interfaces, tool packages 2, 11, and 12.
    https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/primary-secondary-guide.html
    """
    deadline = time.monotonic() + 3.0
    with socket.create_connection((host, 30011), timeout=2) as sock:
        while time.monotonic() < deadline:
            size, kind = struct.unpack('>IB', receive(sock, 5, deadline))
            if not 5 <= size <= 65536:
                raise ValueError('Invalid primary-interface message length')
            data = receive(sock, size - 5, deadline)
            if kind != 16:
                continue
            offset, result = 0, {}
            while offset + 5 <= len(data):
                length, package = struct.unpack_from('>IB', data, offset)
                if length < 5 or offset + length > len(data):
                    raise ValueError('Invalid primary-interface subpackage length')
                body = data[offset + 5:offset + length]
                if package == 11:
                    result.update(zip(
                        ['enabled', 'baud', 'parity', 'stop_bits', 'rx_idle', 'tx_idle'],
                        struct.unpack('>?iiiff', body)))
                elif package == 2:
                    values = struct.unpack('>BBddfBffB', body)
                    result['voltage'] = values[5]
                    result['current_a'] = values[6]
                elif package == 12:
                    result['output_modes'] = list(body)
                offset += length
            if 'enabled' in result and 'voltage' in result:
                return result
    raise TimeoutError('No complete tool configuration received')


def read_register(sock, address, unit=65):
    # Function 03 is read-only. This module exposes no write operation.
    sock.sendall(frame(struct.pack('>BBHH', unit, 3, address, 1)))
    deadline = time.monotonic() + 2.0
    header = receive(sock, 3, deadline)
    if header[0] != unit or header[1] not in (3, 0x83):
        raise ValueError(f'Unexpected response header {header.hex()}; check serial ownership')
    if header[1] == 3 and header[2] != 2:
        raise ValueError(f'Unexpected response byte count {header[2]}')
    packet = header + receive(sock, 2 if header[1] == 0x83 else 4, deadline)
    if crc16(packet[:-2]) != struct.unpack('<H', packet[-2:])[0]:
        raise ValueError('Response CRC mismatch; possible concurrent serial traffic')
    if header[1] == 0x83:
        raise ValueError(f'Modbus exception {header[2]} for register {address}')
    return struct.unpack('>H', packet[3:5])[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.56.101')
    parser.add_argument('--port', type=int, default=54321)
    parser.add_argument('--unit', type=int, choices=[65, 66, 67], default=65)
    parser.add_argument('--tool-io-user-confirmed', action='store_true', required=True)
    args = parser.parse_args()
    config = tool_configuration(args.host)
    print(json.dumps({'tool_configuration': config}), flush=True)
    if not (config['enabled'] and config['baud'] == 1000000
            and config['parity'] == 2 and config['stop_bits'] == 1
            and config['voltage'] == 24):
        raise SystemExit('Actual tool settings do not match RG2 serial requirements')
    with socket.create_connection((args.host, args.port), timeout=2) as sock:
        values = {name: read_register(sock, register, args.unit) for name, register in (
            ('width_raw', 267), ('status', 268),
            ('width_with_offset_raw', 275), ('fingertip_offset_raw', 258))}
    width = signed16(values['width_with_offset_raw']) / 10.0
    valid_feedback = 0 <= width <= 110 and not values['status'] & 0x7C
    print(json.dumps({
        'read_only': True,
        'unit': args.unit,
        'raw_registers': values,
        'width_mm': signed16(values['width_raw']) / 10.0,
        'width_with_offset_mm': width,
        'fingertip_offset_mm': signed16(values['fingertip_offset_raw']) / 10.0,
        'status_register': values['status'],
        'busy': bool(values['status'] & 1),
        'grip_detected': bool(values['status'] & 2),
        'feedback_within_lab_bounds': valid_feedback,
    }, indent=2))
    if not valid_feedback:
        raise SystemExit('Feedback is outside lab bounds; do not command motion')


if __name__ == '__main__':
    main()
