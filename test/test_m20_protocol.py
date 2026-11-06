import json
import math
import struct

import pytest

from m20_protocol import (HEADER_SIZE, SYNC, Frame, MessageIdCounter, StreamDecoder, decode,
                          encode, try_decode)
from m20_protocol import commands as C

T = '2026-11-06 12:00:00'


def test_header_layout():
    frame = encode(2, 21, {'X': 0.5}, msg_id=0x1234, timestamp=T)
    assert frame[:4] == bytes.fromhex('EB91EB90') == SYNC
    body = frame[HEADER_SIZE:]
    assert struct.unpack_from('<H', frame, 4)[0] == len(body)
    assert struct.unpack_from('<H', frame, 6)[0] == 0x1234
    assert frame[8] == 0x01
    assert frame[9:16] == bytes(7)
    assert json.loads(body) == {'PatrolDevice': {'Type': 2, 'Command': 21, 'Time': T,
                                                 'Items': {'X': 0.5}}}


def test_round_trip():
    frame = decode(encode(1101, 5, {'Mode': 1}, msg_id=7, timestamp=T))
    assert frame == Frame(7, 1101, 5, {'Mode': 1},
                          {'Type': 1101, 'Command': 5, 'Time': T, 'Items': {'Mode': 1}})


@pytest.mark.parametrize('args', [
    (True, 1, {}), (1, 1.0, {}), (1, 1, []), (1, 1, {'x': float('nan')}),
])
def test_encode_rejects_bad_input(args):
    with pytest.raises(ValueError):
        encode(*args)


@pytest.mark.parametrize('msg_id', [-1, 0x10000, True])
def test_encode_rejects_bad_msg_id(msg_id):
    with pytest.raises(ValueError):
        encode(1, 1, {}, msg_id=msg_id)


def test_encode_rejects_oversized_body():
    with pytest.raises(ValueError):
        encode(1, 1, {'a': 'x' * 70000})


def _with_header(body, sync=SYNC, fmt=1, reserved=bytes(7), length=None):
    length = len(body) if length is None else length
    return sync + struct.pack('<HHB', length, 0, fmt) + reserved + body


@pytest.mark.parametrize('packet', [
    b'',
    SYNC,
    _with_header(b'{}', sync=bytes.fromhex('EB90EB90')),
    _with_header(b'{}', fmt=2),
    _with_header(b'{}', reserved=b'\x01' + bytes(6)),
    _with_header(b'{}', length=3),
    _with_header(b'[]'),
    _with_header(b'{"PatrolDevice": {"Type": 1, "Command": 1}}'),
    _with_header(b'{"PatrolDevice": {"Type": true, "Command": 1, "Items": {}}}'),
    _with_header(b'{"PatrolDevice": {"Type": 1, "Command": 1, "Items": {"a": NaN}}}'),
    _with_header(b'\xff\xfe'),
])
def test_decode_rejects_invalid(packet):
    with pytest.raises(ValueError):
        decode(packet)
    assert try_decode(packet) is None


def test_stream_decoder_handles_split_and_joined_frames():
    a = encode(100, 100, {}, msg_id=1, timestamp=T)
    b = encode(2, 21, {'X': 0.1}, msg_id=2, timestamp=T)
    d = StreamDecoder()
    stream = a + b
    out = []
    for i in range(0, len(stream), 5):
        out.extend(d.feed(stream[i:i + 5]))
    assert [f.msg_id for f in out] == [1, 2]
    d.eof()


def test_stream_decoder_poisons_on_garbage():
    d = StreamDecoder()
    with pytest.raises(ValueError):
        d.feed(b'\x00' * HEADER_SIZE)
    assert d.failed
    with pytest.raises(ValueError):
        d.feed(encode(100, 100))


def test_stream_decoder_eof_with_partial_frame():
    d = StreamDecoder()
    d.feed(encode(100, 100)[:10])
    with pytest.raises(ValueError):
        d.eof()


def test_message_id_counter_wraps():
    counter = MessageIdCounter(0xFFFE)
    assert [counter(), counter(), counter()] == [0xFFFE, 0xFFFF, 0]


def test_builders():
    assert decode(C.heartbeat()).type == 100
    assert decode(C.set_usage_mode(C.MODE_NAV)).items == {'Mode': 1}
    with pytest.raises(ValueError):
        C.set_usage_mode(3)
    assert decode(C.set_gait(12290)).items == {'GaitParam': 12290}
    assert decode(C.set_gait(C.GAIT_STAIR_V004)).items == {'GaitParam': 14}
    assert decode(C.set_motion_state(1)).items == {'MotionParam': 1}

    f = decode(C.axis_normalized(2.0, -0.5, -3.0))
    assert f.command == 21
    assert f.items == {'X': 1.0, 'Y': -0.5, 'Z': 0.0, 'Roll': 0.0, 'Pitch': 0.0, 'Yaw': -1.0}

    f = decode(C.velocity_si(0.3, 0.0, 0.2))
    assert f.command == 25
    assert f.items['X'] == 0.3 and f.items['Yaw'] == 0.2
    with pytest.raises(ValueError):
        C.velocity_si(float('inf'), 0, 0)


def test_normalize_velocity():
    assert C.normalize_velocity((1.0, 2.0, -4.0)) == (0.5, 1.0, -1.0)
    with pytest.raises(ValueError):
        C.normalize_velocity((1, 1, 1), (0.0, 1.0, 1.0))


@pytest.mark.parametrize('yaw,expected', [
    (0.0, 0.0), (1.0, 1.0), (-1.0, -1.0), (math.pi, -math.pi),
    (3 * math.pi / 2, -math.pi / 2), (-7.0, -7.0 + 2 * math.pi),
])
def test_normalize_yaw(yaw, expected):
    assert C.normalize_yaw(yaw) == pytest.approx(expected)


@pytest.mark.parametrize('profile,mode,aux,expected', [
    ('auto', C.MODE_NAV, False, 25),
    ('auto', C.MODE_REGULAR, False, 21),
    ('auto', C.MODE_AUX, False, None),
    ('auto', C.MODE_AUX, True, 21),
    ('si', C.MODE_NAV, False, 25),
    ('si', C.MODE_REGULAR, False, None),
    ('normalized', C.MODE_NAV, False, None),
    ('normalized', C.MODE_REGULAR, False, 21),
    ('normalized', C.MODE_AUX, True, 21),
])
def test_select_velocity_command(profile, mode, aux, expected):
    assert C.select_velocity_command(profile, mode, aux) == expected
