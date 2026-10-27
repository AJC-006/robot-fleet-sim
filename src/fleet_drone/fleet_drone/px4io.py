"""Shared PX4 <-> ROS helpers: frame conversions, versioned uXRCE topic discovery, QoS."""
import math

import numpy as np
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

# PX4 publishes /fmu/out/* best effort; inputs are read by PX4 regardless of reliability.
QOS_PX4_OUT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
QOS_PX4_IN = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=5)

# gz NavSat turns world metres into lat/lon on the WGS84 ellipsoid; PX4 (MapProjection in EKF2) projects lat/lon
# back onto a sphere of radius 6371 km. PX4 local NED metres are therefore k * world metres, k_n = R/M, k_e = R/N
# (meridian / prime-vertical radii at the world's latitude). At the project's worlds' 47.4 deg N east is 0.29 % short:
# 0.7 m over the keep's 250 m, far more than the waypoint tolerance.
R_PX4 = 6371000.0
WGS84_A, WGS84_E2 = 6378137.0, 6.69437999014e-3


def geo_scale(lat_deg, model="EARTH_WGS84"):
    """[k_north, k_east, 1]: PX4 local NED metres per gz world metre ([1, 1, 1] unless the world is WGS84)."""
    if str(model).upper() != "EARTH_WGS84":
        return np.ones(3)
    w = 1.0 - WGS84_E2 * math.sin(math.radians(lat_deg)) ** 2
    n = WGS84_A / math.sqrt(w)
    m = WGS84_A * (1.0 - WGS84_E2) / w ** 1.5
    return np.array([R_PX4 / m, R_PX4 / n, 1.0])


_S = math.sqrt(0.5)
Q_ENU_NED = np.array([0.0, _S, _S, 0.0])     # w, x, y, z
Q_FRD_FLU = np.array([0.0, 1.0, 0.0, 0.0])


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def q_ned_frd_to_enu_flu(q):
    """PX4 attitude (FRD body -> NED, [w,x,y,z]) to ROS (FLU body -> ENU)."""
    r = qmul(qmul(Q_ENU_NED, np.asarray(q, float)), Q_FRD_FLU)
    if r[0] < 0:
        r = -r
    return r / np.linalg.norm(r)


def ned_to_enu(v):
    return np.array([v[1], v[0], -v[2]])


def enu_to_ned(v):
    return np.array([v[1], v[0], -v[2]])


def frd_to_flu(v):
    return np.array([v[0], -v[1], -v[2]])


def qrot(q, v):
    """Rotate vector v by quaternion q ([w,x,y,z])."""
    qv = np.array([0.0, v[0], v[1], v[2]])
    qc = np.array([q[0], -q[1], -q[2], -q[3]])
    return qmul(qmul(q, qv), qc)[1:]


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def enu_yaw_to_ned(yaw):
    """ENU yaw (from east, CCW) to PX4 NED heading (from north, CW)."""
    return wrap(math.pi / 2 - yaw)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def find_topic(node, base):
    """Resolve a PX4 uXRCE topic that may carry a _v<N> version suffix (e.g. /fmu/out/vehicle_status_v4)."""
    names = [n for n, _ in node.get_topic_names_and_types()]
    if base in names:
        return base
    for n in sorted(names):
        if n.startswith(base + "_v") and n[len(base) + 2:].isdigit():
            return n
    return None
