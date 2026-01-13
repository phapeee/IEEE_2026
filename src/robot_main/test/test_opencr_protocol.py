import os
import secrets
import time
from dataclasses import dataclass

import pytest

serial = pytest.importorskip("serial")

MSG_HEADER_1 = 0xAA
MSG_HEADER_2 = 0x55
MSG_TYPE_CMD_VEL = 0x01
MSG_TYPE_PING = 0x02
MSG_TYPE_ECHO = 0x03
MSG_TYPE_STATUS = 0x10
STATUS_PAYLOAD_LEN = 17  # 1 status byte + 4 floats
MAX_PAYLOAD_LEN = 32


@dataclass
class Frame:
    msg_type: int
    seq: int
    payload: bytes
    crc: int


def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    length = len(payload)
    body = bytes([msg_type, length, seq]) + payload
    crc = crc16_ccitt(body)
    return bytes([MSG_HEADER_1, MSG_HEADER_2]) + body + crc.to_bytes(2, "little")


def _read_frame(ser: serial.Serial, timeout: float = 1.0) -> Frame:
    deadline = time.time() + timeout
    state = "h1"
    msg_type = 0
    length = 0
    seq = 0
    payload = bytearray()

    while time.time() < deadline:
        remaining = max(0.0, deadline - time.time())
        byte = ser.read(1 if remaining > 0 else 0)
        if not byte:
            continue

        value = byte[0]

        if state == "h1":
            state = "h2" if value == MSG_HEADER_1 else "h1"
            continue

        if state == "h2":
            if value == MSG_HEADER_2:
                state = "type"
            elif value != MSG_HEADER_1:
                state = "h1"
            continue

        if state == "type":
            msg_type = value
            state = "len"
            continue

        if state == "len":
            length = value
            if length > MAX_PAYLOAD_LEN:
                state = "h1"
                continue
            state = "seq"
            continue

        if state == "seq":
            seq = value
            payload = bytearray()
            state = "payload" if length else "crc_lo"
            continue

        if state == "payload":
            payload.append(value)
            if len(payload) == length:
                state = "crc_lo"
            continue

        if state == "crc_lo":
            crc_low = value
            state = "crc_hi"
            continue

        if state == "crc_hi":
            crc_received = (value << 8) | crc_low
            crc_expected = crc16_ccitt(bytes([msg_type, length, seq]) + payload)
            if crc_expected == crc_received:
                return Frame(msg_type, seq, bytes(payload), crc_expected)
            state = "h1"

    raise TimeoutError("Timed out waiting for a frame")


def _wait_for_frame(ser: serial.Serial, expected_type: int, expected_seq: int | None = None, timeout: float = 2.0) -> Frame:
    deadline = time.time() + timeout
    while time.time() < deadline:
        frame = _read_frame(ser, timeout=deadline - time.time())
        if frame.msg_type == MSG_TYPE_STATUS:
            assert_status_frame(frame)
            if expected_type != MSG_TYPE_STATUS:
                continue
        if frame.msg_type != expected_type:
            continue
        if expected_seq is not None and frame.seq != expected_seq:
            continue
        return frame
    raise AssertionError(f"Did not receive expected frame type {expected_type:#04x}")


def _assert_no_response(ser: serial.Serial, blocked_type: int, seq: int, window: float = 1.0) -> None:
    deadline = time.time() + window
    while time.time() < deadline:
        try:
            frame = _read_frame(ser, timeout=deadline - time.time())
        except TimeoutError:
            return
        if frame.msg_type == MSG_TYPE_STATUS:
            assert_status_frame(frame)
            continue
        if frame.msg_type == blocked_type and frame.seq == seq:
            pytest.fail(f"Unexpected response of type {blocked_type:#04x} for seq {seq}")


@pytest.fixture(scope="module")
def serial_port():
    port = os.getenv("OPENCR_SERIAL_PORT", "/dev/ttyACM0")
    baud = int(os.getenv("OPENCR_SERIAL_BAUD", "115200"))

    try:
        ser = serial.Serial(port, baudrate=baud, timeout=0.05)
    except serial.SerialException as exc:
        pytest.skip(f"Serial port {port} unavailable: {exc}")

    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.1)

    yield ser
    ser.close()


def assert_status_frame(frame: Frame) -> None:
    assert frame.msg_type == MSG_TYPE_STATUS
    assert len(frame.payload) == STATUS_PAYLOAD_LEN


def test_ping_roundtrip(serial_port):
    seq = secrets.randbits(8)
    serial_port.write(build_frame(MSG_TYPE_PING, seq))
    reply = _wait_for_frame(serial_port, MSG_TYPE_PING, expected_seq=seq)
    assert reply.payload == b""


def test_echo_small_payload(serial_port):
    seq = secrets.randbits(8)
    payload = b"hello"
    serial_port.write(build_frame(MSG_TYPE_ECHO, seq, payload))
    reply = _wait_for_frame(serial_port, MSG_TYPE_ECHO, expected_seq=seq)
    assert reply.payload == payload


def test_echo_random_payload(serial_port):
    seq = secrets.randbits(8)
    payload_len = secrets.randbelow(16) + 1
    payload = secrets.token_bytes(payload_len)
    serial_port.write(build_frame(MSG_TYPE_ECHO, seq, payload))
    reply = _wait_for_frame(serial_port, MSG_TYPE_ECHO, expected_seq=seq)
    assert reply.payload == payload


def test_status_frames_well_formed(serial_port):
    seen = 0
    deadline = time.time() + 4.0
    while seen < 3 and time.time() < deadline:
        frame = _wait_for_frame(serial_port, MSG_TYPE_STATUS, timeout=deadline - time.time())
        assert_status_frame(frame)
        seen += 1
    assert seen >= 3


def test_bad_crc_is_ignored(serial_port):
    seq = secrets.randbits(8)
    frame = bytearray(build_frame(MSG_TYPE_PING, seq))
    frame[-1] ^= 0x01
    serial_port.write(frame)

    _assert_no_response(serial_port, MSG_TYPE_PING, seq, window=1.0)

    status_frame = _wait_for_frame(serial_port, MSG_TYPE_STATUS, timeout=2.0)
    assert_status_frame(status_frame)
