"""Protocol validation without contacting physical hardware."""

import struct
import unittest

from lab_rg2_read import crc16, frame, read_register, signed16


class FragmentedSocket:
    def __init__(self, response):
        self.response = bytearray(response)
        self.sent = None

    def sendall(self, request):
        self.sent = request

    def settimeout(self, timeout):
        pass

    def recv(self, count):
        if not self.response:
            return b''
        return bytes([self.response.pop(0)])


class ReadProtocol(unittest.TestCase):
    def test_standard_crc_vector(self):
        self.assertEqual(crc16(bytes.fromhex('01030000000a')), 0xCDC5)

    def test_fragmented_response_and_read_only_request(self):
        sock = FragmentedSocket(frame(bytes.fromhex('4103020220')))
        self.assertEqual(read_register(sock, 267), 544)
        self.assertEqual(sock.sent[:6], struct.pack('>BBHH', 65, 3, 267, 1))

    def test_corrupted_frame_rejected(self):
        packet = bytearray(frame(bytes.fromhex('4103020220')))
        packet[-1] ^= 1
        with self.assertRaisesRegex(ValueError, 'CRC'):
            read_register(FragmentedSocket(packet), 267)

    def test_wrong_device_rejected(self):
        with self.assertRaisesRegex(ValueError, 'header'):
            read_register(FragmentedSocket(frame(bytes.fromhex('4203020220'))), 267)

    def test_selected_unit_response(self):
        sock = FragmentedSocket(frame(bytes.fromhex('4303020220')))
        self.assertEqual(read_register(sock, 267, 67), 544)
        self.assertEqual(sock.sent[0], 67)

    def test_negative_register_is_not_positive_width(self):
        self.assertEqual(signed16(0xFF55), -171)

    def test_modbus_exception_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Modbus exception 2'):
            read_register(FragmentedSocket(frame(bytes.fromhex('418302'))), 267)

    def test_truncated_response_rejected(self):
        with self.assertRaises(ConnectionError):
            read_register(FragmentedSocket(bytes.fromhex('41030202')), 267)


if __name__ == '__main__':
    unittest.main()
