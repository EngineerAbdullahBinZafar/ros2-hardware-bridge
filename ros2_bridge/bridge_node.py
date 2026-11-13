"""
ros2_hardware_bridge / bridge_node.py
Autonomous Systems & Control Engineering Bridge
Protocol: AUTOSAR E2E Profile 1 (SAE J1850 CRC-8, 4-bit Sequence Counter)
Author: Engineer Abdullah Bin Zafar
License: Apache-2.0
"""

import struct
import sys
from typing import Optional, Tuple

CRC8_SAE_J1850_TABLE = [
    0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
    0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E, 0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
    0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
    0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
    0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40, 0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
    0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
    0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
    0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A, 0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
    0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
    0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
    0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2, 0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
    0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F, 0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
    0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
    0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB, 0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
    0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
    0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4
]

def compute_autosar_crc8(data: bytes, data_id: Optional[int] = None) -> int:
    crc = 0xFF
    if data_id is not None:
        id_lo = data_id & 0xFF
        id_hi = (data_id >> 8) & 0xFF
        crc = CRC8_SAE_J1850_TABLE[crc ^ id_lo]
        crc = CRC8_SAE_J1850_TABLE[crc ^ id_hi]
    for b in data:
        crc = CRC8_SAE_J1850_TABLE[crc ^ b]
    return crc ^ 0xFF


def cobs_encode(data: bytes) -> bytes:
    """Encode bytes using Consistent Overhead Byte Stuffing (COBS) without 0x00 delimiter."""
    out = bytearray([0])
    code_idx = 0
    code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code = 1
            code_idx = len(out)
            out.append(0)
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code = 1
                code_idx = len(out)
                out.append(0)
    out[code_idx] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """Decode COBS-encoded bytes back to raw frame. Raises ValueError on malformed framing."""
    if not data:
        raise ValueError("Empty COBS frame")
    out = bytearray()
    idx = 0
    n = len(data)
    while idx < n:
        code = data[idx]
        if code == 0 or idx + code > n + 1:
            raise ValueError("Malformed COBS code byte")
        idx += 1
        for _ in range(1, code):
            if idx >= n:
                raise ValueError("Truncated COBS block")
            out.append(data[idx])
            idx += 1
        if code < 0xFF and idx < n:
            out.append(0)
    return bytes(out)


def encode_e2e_frame(
    msg_id: int,
    seq_ctr: int,
    payload: bytes,
    data_id: Optional[int] = None,
    use_cobs: bool = False,
) -> bytes:
    """Serialize an AUTOSAR E2E Profile 1 frame (optionally Data-ID authenticated and COBS-delimited)."""
    if len(payload) > 64:
        raise ValueError("Payload exceeds 64-byte maximum")
    header_and_payload = bytes([msg_id & 0xFF, seq_ctr & 0x0F, len(payload)]) + payload
    crc = compute_autosar_crc8(header_and_payload, data_id=data_id)
    raw = b"\xAA\x55" + header_and_payload + bytes([crc])
    if use_cobs:
        return cobs_encode(raw) + b"\x00"
    return raw


class PacketDecoder:
    """
    AUTOSAR E2E Profile 1 State Machine Packet Decoder.
    Frame: [0xAA, 0x55, MSG_ID, SEQ_CTR, LENGTH, PAYLOAD (NB), CRC8]
    Supports 16-bit Data ID authentication, max_delta_counter tracking, and COBS framing.
    """
    STATE_HEADER_1 = 0
    STATE_HEADER_2 = 1
    STATE_MSG_ID = 2
    STATE_SEQ_CTR = 3
    STATE_LENGTH = 4
    STATE_PAYLOAD = 5
    STATE_CRC8 = 6

    def __init__(self, data_id: Optional[int] = None, max_delta_counter: int = 2):
        self.state = self.STATE_HEADER_1
        self.raw_seq_byte = 0
        self.msg_id = 0
        self.seq_ctr = 0
        self.length = 0
        self.payload = bytearray()
        self.crc8 = 0
        self.data_id = data_id
        self.max_delta_counter = max(1, min(14, int(max_delta_counter)))
        self.seq_initialized = False
        self.last_seq_ctr = 0
        self.last_status = "IDLE"
        self.lost_frames = 0
        self.frames_valid = 0
        self.frames_crc_err = 0
        self.frames_repeated = 0
        self.frames_seq_lost = 0
        self.resync_events = 0
        self._cobs_buf = bytearray()

    def _evaluate_sequence(self, seq: int) -> str:
        self.lost_frames = 0
        if not self.seq_initialized:
            self.seq_initialized = True
            self.last_seq_ctr = seq
            self.frames_valid += 1
            return "INITIAL"
        expected = (self.last_seq_ctr + 1) & 0x0F
        delta = (seq - expected) & 0x0F
        if delta == 0:
            self.last_seq_ctr = seq
            self.frames_valid += 1
            return "OK"
        if delta == 15:
            self.frames_repeated += 1
            return "REPEATED"
        self.last_seq_ctr = seq
        self.lost_frames = delta
        self.frames_seq_lost += delta
        if delta > self.max_delta_counter:
            self.resync_events += 1
            return "DESYNC"
        return "WRONG_SEQUENCE"

    def parse_byte(self, b: int) -> Optional[Tuple[int, int, bytes]]:
        if self.state == self.STATE_HEADER_1:
            if b == 0xAA:
                self.state = self.STATE_HEADER_2

        elif self.state == self.STATE_HEADER_2:
            if b == 0x55:
                self.state = self.STATE_MSG_ID
            elif b == 0xAA:
                self.state = self.STATE_HEADER_2
            else:
                self.resync_events += 1
                self.state = self.STATE_HEADER_1

        elif self.state == self.STATE_MSG_ID:
            self.msg_id = b
            self.state = self.STATE_SEQ_CTR

        elif self.state == self.STATE_SEQ_CTR:
            self.raw_seq_byte = b
            self.seq_ctr = b & 0x0F
            self.state = self.STATE_LENGTH

        elif self.state == self.STATE_LENGTH:
            self.length = b
            self.payload = bytearray()
            if self.length == 0:
                self.state = self.STATE_CRC8
            elif self.length > 64:
                self.frames_crc_err += 1
                self.resync_events += 1
                self.last_status = "FRAME_ERROR"
                self.state = self.STATE_HEADER_2 if b == 0xAA else self.STATE_HEADER_1
            else:
                self.state = self.STATE_PAYLOAD

        elif self.state == self.STATE_PAYLOAD:
            self.payload.append(b)
            if len(self.payload) == self.length:
                self.state = self.STATE_CRC8

        elif self.state == self.STATE_CRC8:
            self.crc8 = b
            self.state = self.STATE_HEADER_2 if b == 0xAA else self.STATE_HEADER_1

            header_and_payload = bytes([self.msg_id, self.raw_seq_byte, self.length]) + bytes(self.payload)
            calc_crc = compute_autosar_crc8(header_and_payload, data_id=self.data_id)

            if calc_crc == self.crc8:
                self.last_status = self._evaluate_sequence(self.seq_ctr)
                if self.last_status in ("INITIAL", "OK", "WRONG_SEQUENCE"):
                    return (self.msg_id, self.seq_ctr, bytes(self.payload))
            else:
                self.frames_crc_err += 1
                self.last_status = "CRC_ERROR"

        return None

    def parse_cobs_byte(self, b: int) -> Optional[Tuple[int, int, bytes]]:
        """Feed one byte from a 0x00-delimited COBS E2E stream."""
        if b != 0:
            if len(self._cobs_buf) < 72:
                self._cobs_buf.append(b)
            else:
                self._cobs_buf.clear()
                self.last_status = "FRAME_ERROR"
                self.resync_events += 1
            return None

        if not self._cobs_buf:
            return None

        encoded = bytes(self._cobs_buf)
        self._cobs_buf.clear()
        try:
            raw = cobs_decode(encoded)
        except ValueError:
            self.frames_crc_err += 1
            self.resync_events += 1
            self.last_status = "FRAME_ERROR"
            return None

        if len(raw) < 6 or raw[0] != 0xAA or raw[1] != 0x55 or len(raw) != 6 + raw[4]:
            self.frames_crc_err += 1
            self.resync_events += 1
            self.last_status = "FRAME_ERROR"
            self.state = self.STATE_HEADER_1
            return None

        self.state = self.STATE_HEADER_1
        out = None
        for rb in raw:
            res = self.parse_byte(rb)
            if res is not None:
                out = res
        self.state = self.STATE_HEADER_1
        return out
