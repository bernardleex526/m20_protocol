"""DEEP Robotics M20 ``basic_server`` frame codec.

Frame layout (16-byte header + UTF-8 JSON ASDU)::

    0..3   sync word   EB 91 EB 90
    4..5   uint16 LE   ASDU length in bytes
    6..7   uint16 LE   message id
    8      uint8       format, 0x01 = JSON
    9..15  7 bytes     reserved, zero

The ASDU is ``{"PatrolDevice": {"Type": int, "Command": int, "Time": str,
"Items": object}}``.

The sync word EB 91 EB 90 matches the vendor manual header table and is what
the ``m20_chase`` and ``deamon`` bridges were
validated against on GOS104.
"""

import json
import struct
import time
from typing import Any, Dict, List, NamedTuple, Optional

SYNC = bytes.fromhex('EB91EB90')
FMT_JSON = 0x01
HEADER = struct.Struct('<4sHHB7s')
HEADER_SIZE = HEADER.size  # 16
MAX_BODY_SIZE = 65535
MAX_FRAME_SIZE = HEADER_SIZE + MAX_BODY_SIZE
_RESERVED = bytes(7)


class Frame(NamedTuple):
    msg_id: int
    type: int
    command: int
    items: Dict[str, Any]
    raw: Dict[str, Any]  # the full PatrolDevice object, including Time


def now_str() -> str:
    """Local time in the ``YYYY-MM-DD HH:MM:SS`` format the robot expects."""
    return time.strftime('%Y-%m-%d %H:%M:%S')


def _is_int(value) -> bool:
    return type(value) is int  # rejects bool


def encode(type_: int, command: int, items: Optional[dict] = None,
           msg_id: int = 0, timestamp: Optional[str] = None) -> bytes:
    """Encode one frame. Raises ``ValueError`` on invalid input."""
    if not _is_int(type_) or not _is_int(command):
        raise ValueError('Type and Command must be integers')
    if items is None:
        items = {}
    if not isinstance(items, dict):
        raise ValueError('Items must be an object')
    if not _is_int(msg_id) or not 0 <= msg_id <= 0xFFFF:
        raise ValueError('message id outside uint16')
    body = json.dumps(
        {'PatrolDevice': {'Type': type_, 'Command': command,
                          'Time': timestamp or now_str(), 'Items': items}},
        ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode('utf-8')
    if len(body) > MAX_BODY_SIZE:
        raise ValueError('ASDU exceeds 65535 bytes')
    return HEADER.pack(SYNC, len(body), msg_id, FMT_JSON, _RESERVED) + body


def _body_length(buf) -> int:
    sync, length, _msg_id, fmt, reserved = HEADER.unpack_from(buf)
    if sync != SYNC or fmt != FMT_JSON or reserved != _RESERVED:
        raise ValueError('invalid frame header')
    return length


def _reject_constant(value):
    raise ValueError(f'invalid JSON constant: {value}')


def decode(buf) -> Frame:
    """Decode exactly one complete frame. Raises ``ValueError`` if invalid."""
    buf = bytes(buf)
    if len(buf) < HEADER_SIZE:
        raise ValueError('truncated header')
    length = _body_length(buf)
    if len(buf) != HEADER_SIZE + length:
        raise ValueError('frame length does not match header')
    try:
        obj = json.loads(buf[HEADER_SIZE:].decode('utf-8'),
                         parse_constant=_reject_constant)
    except (UnicodeError, RecursionError) as exc:
        raise ValueError(f'invalid ASDU: {exc}') from exc
    pd = obj.get('PatrolDevice') if isinstance(obj, dict) else None
    if not isinstance(pd, dict):
        raise ValueError('missing PatrolDevice')
    if (not _is_int(pd.get('Type')) or not _is_int(pd.get('Command'))
            or not isinstance(pd.get('Items'), dict)):
        raise ValueError('invalid ASDU fields')
    msg_id = struct.unpack_from('<H', buf, 6)[0]
    return Frame(msg_id, pd['Type'], pd['Command'], pd['Items'], pd)


def try_decode(buf) -> Optional[Frame]:
    """Like :func:`decode` but returns ``None`` for any invalid frame."""
    try:
        return decode(buf)
    except (ValueError, struct.error):
        return None


class StreamDecoder:
    """Incremental TCP decoder.

    An invalid frame poisons the decoder: it never tries to resync inside a
    stream, the caller must drop the connection.
    """

    def __init__(self):
        self._buffer = bytearray()
        self._failed = False

    @property
    def failed(self) -> bool:
        return self._failed

    def feed(self, data) -> List[Frame]:
        if self._failed:
            raise ValueError('decoder has failed')
        self._buffer.extend(data)
        frames = []
        try:
            while len(self._buffer) >= HEADER_SIZE:
                size = HEADER_SIZE + _body_length(self._buffer)
                if len(self._buffer) < size:
                    break
                frames.append(decode(self._buffer[:size]))
                del self._buffer[:size]
            return frames
        except ValueError:
            self._failed = True
            self._buffer.clear()
            raise

    def eof(self) -> None:
        """Call when the peer closes; raises if a partial frame is pending."""
        if self._failed or self._buffer:
            self._failed = True
            self._buffer.clear()
            raise ValueError('truncated TCP frame')


class MessageIdCounter:
    """Wrapping uint16 message id generator."""

    def __init__(self, start: int = 0):
        self._next = start & 0xFFFF

    def __call__(self) -> int:
        value = self._next
        self._next = (self._next + 1) & 0xFFFF
        return value
