"""basic_server message types, commands and ``Items`` builders.

Sources: the vendor "运动控制（basic_server 协议）" page (software >= V1.1.7,
as summarised in ``deamon``) for motion commands and the per-gait speed table,
and the Lynx M20 Software Development Manual V0.0.4-0 (cited by ``2.5d_ccplan``)
for heartbeat, usage mode, BasicStatus and the older ``GaitParam`` 1 / 14 values.
Navigation, localization and error-code tables are deliberately not included.

Gait ids differ between firmware generations, so the builders here do not
reject unknown gait values; callers pick the table that matches their robot.
"""

import math
from typing import Iterable, Optional, Tuple

from .frame import encode

# --------------------------------------------------------------- type / command
T_HEARTBEAT, C_HEARTBEAT = 100, 100
T_MOTION = 2
C_AXIS_NORMALIZED = 21  # normalized [-1, 1] per axis; regular / auxiliary mode
C_MOTION_STATE = 22
C_GAIT = 23
C_VELOCITY_SI = 25      # m/s, rad/s; navigation mode only
T_SET_USAGE_MODE, C_SET_USAGE_MODE = 1101, 5
T_STATUS, C_BASIC_STATUS = 1002, 6   # Items.BasicStatus (manual section 1.3.1)

DEFAULT_PORT = 30000

# ------------------------------------------------------------------ usage modes
MODE_REGULAR = 0
MODE_NAV = 1
MODE_AUX = 2
MODE_NAMES = {MODE_REGULAR: 'regular', MODE_NAV: 'navigation', MODE_AUX: 'auxiliary'}

# ------------------------------------------------------------------------ gaits
#: Software >= V1.1.7: gait id -> (name, x_min, y_min, yaw_min, x_max, y_max, yaw_max)
GAIT_TABLE_V117 = {
    4097: ('standard-basic', 0.20, 0.35, 0.50, 2.0, 1.0, 2.0),
    4099: ('standard-stairs', 0.15, 0.30, 0.40, 2.0, 1.0, 2.0),
    12290: ('agile-flat', 0.15, 0.25, 0.35, 2.0, 1.0, 1.5),
    12291: ('agile-stairs', 0.15, 0.30, 0.40, 2.0, 1.0, 2.0),
}
#: Manual V0.0.4-0 GaitParam values.
GAIT_BASIC_V004 = 1
GAIT_STAIR_V004 = 14

#: Largest documented per-axis speeds; used as Cmd=21 full scale. Verify on robot.
DEFAULT_AXIS_MAX = (2.0, 1.0, 2.0)

# ---------------------------------------------------------------------- helpers
def _finite(values: Iterable[float]) -> Tuple[float, ...]:
    out = tuple(float(v) for v in values)
    if not all(math.isfinite(v) for v in out):
        raise ValueError('nonfinite value')
    return out


def normalize_yaw(yaw: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    (yaw,) = _finite((yaw,))
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi


def normalize_velocity(velocity, axis_max=DEFAULT_AXIS_MAX) -> Tuple[float, float, float]:
    """SI (vx, vy, wz) -> Cmd=21 ratios clamped to [-1, 1]."""
    out = []
    for value, full_scale in zip(_finite(velocity), _finite(axis_max)):
        if full_scale <= 0.0:
            raise ValueError('axis maximum must be positive')
        out.append(max(-1.0, min(1.0, value / full_scale)))
    return tuple(out)


def _axes(x, y, yaw):
    x, y, yaw = _finite((x, y, yaw))
    return {'X': x, 'Y': y, 'Z': 0.0, 'Roll': 0.0, 'Pitch': 0.0, 'Yaw': yaw}


# --------------------------------------------------------------- frame builders
def heartbeat(msg_id: int = 0) -> bytes:
    return encode(T_HEARTBEAT, C_HEARTBEAT, {}, msg_id)


def set_usage_mode(mode: int, msg_id: int = 0) -> bytes:
    if type(mode) is not int or mode not in MODE_NAMES:
        raise ValueError('usage mode must be 0 (regular), 1 (navigation) or 2 (auxiliary)')
    return encode(T_SET_USAGE_MODE, C_SET_USAGE_MODE, {'Mode': mode}, msg_id)


def set_motion_state(param: int, msg_id: int = 0) -> bytes:
    return encode(T_MOTION, C_MOTION_STATE, {'MotionParam': int(param)}, msg_id)


def set_gait(gait: int, msg_id: int = 0) -> bytes:
    if type(gait) is not int:
        raise ValueError('gait must be an integer')
    return encode(T_MOTION, C_GAIT, {'GaitParam': gait}, msg_id)


def axis_normalized(x: float, y: float, yaw: float, msg_id: int = 0) -> bytes:
    """Cmd=21 with ratios; values are clamped to [-1, 1], Z/Roll/Pitch forced to 0."""
    x, y, yaw = (max(-1.0, min(1.0, v)) for v in _finite((x, y, yaw)))
    return encode(T_MOTION, C_AXIS_NORMALIZED, _axes(x, y, yaw), msg_id)


def velocity_si(vx: float, vy: float, wz: float, msg_id: int = 0) -> bytes:
    """Cmd=25 in m/s and rad/s (navigation usage mode only)."""
    return encode(T_MOTION, C_VELOCITY_SI, _axes(vx, vy, wz), msg_id)


def select_velocity_command(profile: str, usage_mode: int,
                            allow_auxiliary: bool = False) -> Optional[int]:
    """Pick Cmd=25 or Cmd=21 for a profile ('auto' | 'si' | 'normalized').

    Returns ``None`` when the vendor documentation does not allow the pair.
    """
    if profile not in ('auto', 'si', 'normalized'):
        raise ValueError('unknown control profile: ' + str(profile))
    if usage_mode == MODE_NAV and profile in ('auto', 'si'):
        return C_VELOCITY_SI
    if profile == 'si':
        return None
    if usage_mode == MODE_REGULAR or (usage_mode == MODE_AUX and allow_auxiliary):
        return C_AXIS_NORMALIZED
    return None
