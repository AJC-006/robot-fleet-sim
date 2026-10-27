"""wyvern_gz_capture: headless chase-camera frames of the wyvern in Gazebo.

Needs the camera model that drone.launch.py inserts into its world copy with capture:=true (a gravity-free,
collision-free link carrying a camera sensor on gz topic /wyvern_capture/image). Pure gz-transport, no ROS:

  * follows /model/wyvern/pose (PosePublisher in the wyvern model);
  * every `--period` seconds of sim time, once the drone is airborne (> `--min-alt` above its start), moves the
    camera with /world/<w>/set_pose to a point at the next `--dist` (m, cycled) from the drone at the next azimuth
    of `--az` (degrees, relative to the drone's heading; 0 = straight behind) and `--elev` degrees up, aimed at it;
  * saves the second camera frame after the move as <out>/<prefix>_NNN.png (sensors only render while
    subscribed, so frames exist only while this tool runs).

GZ_PARTITION must match the simulation's. set_pose goes through the `gz service` CLI: an in-process blocking
request from Python timed out every time while the image/pose callbacks were busy (lab at RTF 0.9).
"""
import argparse
import math
import os
import shutil
import subprocess
import threading
import time

import numpy as np
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node
from google.protobuf import text_format

CAM_MODEL = "wyvern_capture_cam"
CAM_TOPIC = "/wyvern_capture/image"


def camera_model_sdf(width=1600, height=900, hfov=1.05, rate=10.0):
    """SDF snippet for the capture camera (inserted into the world copy by the launch file)."""
    return f"""
    <model name="{CAM_MODEL}">
      <pose>0 0 -50 0 0 0</pose>
      <link name="link">
        <gravity>false</gravity>
        <inertial><mass>0.01</mass><inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <sensor name="cam" type="camera">
          <camera>
            <horizontal_fov>{hfov}</horizontal_fov>
            <image><width>{width}</width><height>{height}</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>800</far></clip>
            <anti_aliasing>4</anti_aliasing>
          </camera>
          <always_on>1</always_on>
          <update_rate>{rate}</update_rate>
          <topic>{CAM_TOPIC}</topic>
        </sensor>
      </link>
    </model>
"""


def quat_look(frm, to):
    """Quaternion (w, x, y, z) of a camera at frm looking at to (camera +x forward, z up)."""
    d = np.asarray(to, float) - np.asarray(frm, float)
    yaw = math.atan2(d[1], d[0])
    pitch = -math.atan2(d[2], math.hypot(d[0], d[1]))
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    return (cy * cp, -sy * sp, cy * sp, sy * cp)


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class Capture:
    def __init__(self, a):
        self.a = a
        self.node = Node()
        self.lock = threading.Lock()
        self.pose = None
        self.stamp = None
        self.frames = 0
        self.want = None
        self.saved = 0
        os.makedirs(a.out, exist_ok=True)
        self.node.subscribe(Pose, "/model/wyvern/pose", self._pose)
        self.node.subscribe(Image, CAM_TOPIC, self._img)

    def _pose(self, m):
        with self.lock:
            self.pose = (np.array([m.position.x, m.position.y, m.position.z]),
                         (m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z))
            self.stamp = m.header.stamp.sec + 1e-9 * m.header.stamp.nsec

    def _img(self, m):
        with self.lock:
            self.frames += 1
            if self.want is None or self.frames < self.want:
                return
            self.want = None
        arr = np.frombuffer(m.data, np.uint8)
        try:
            arr = arr.reshape(m.height, m.width, 3)
        except ValueError:
            return
        from PIL import Image as PImage
        f = os.path.join(self.a.out, "%s_%03d.png" % (self.a.prefix, self.saved))
        PImage.fromarray(arr.copy()).save(f)
        self.saved += 1
        print("saved %s (sim %.1f s)" % (f, self.stamp or 0), flush=True)

    def place(self, pos, q, az_deg, dist):
        a = self.a
        yaw = yaw_of(q) + math.pi + math.radians(az_deg)
        el = math.radians(a.elev)
        cam = pos + dist * np.array([math.cos(el) * math.cos(yaw), math.cos(el) * math.sin(yaw), math.sin(el)])
        cam[2] = max(cam[2], a.min_cam_z)
        look = pos + np.array([0.0, 0.0, -0.05])
        w, x, y, z = quat_look(cam, look)
        req = Pose()
        req.name = CAM_MODEL
        req.position.x, req.position.y, req.position.z = (float(v) for v in cam)
        req.orientation.w, req.orientation.x, req.orientation.y, req.orientation.z = w, x, y, z
        svc = "/world/%s/set_pose" % a.world
        gz = shutil.which("gz")
        for _ in range(3):
            if gz:
                try:
                    out = subprocess.run([gz, "service", "-s", svc, "--reqtype", "gz.msgs.Pose", "--reptype",
                                          "gz.msgs.Boolean", "--timeout", "3000", "--req",
                                          text_format.MessageToString(req, as_one_line=True)],
                                         capture_output=True, text=True, timeout=15).stdout
                except subprocess.TimeoutExpired:
                    out = ""
                if "data: true" in out:
                    return True
            else:
                ok, rep = self.node.request(svc, req, Pose, Boolean, 2000)
                if ok and rep.data:
                    return True
        return False

    def run(self):
        a = self.a
        t_wall0 = time.time()
        z0 = None
        last = None
        i = 0
        while time.time() - t_wall0 < a.timeout and self.saved < a.max_frames:
            time.sleep(0.05)
            with self.lock:
                p, st = self.pose, self.stamp
            if p is None:
                continue
            if z0 is None:
                z0 = p[0][2]
            if p[0][2] - z0 < a.min_alt or (last is not None and st - last < a.period):
                continue
            az, dist = a.az[i % len(a.az)], a.dist[i % len(a.dist)]
            i += 1
            if not self.place(p[0], p[1], az, dist):
                print("set_pose failed", flush=True)
                continue
            last = st
            with self.lock:
                self.want = self.frames + 2
            t = time.time()
            while self.want is not None and time.time() - t < 10:
                time.sleep(0.02)
        print("captured %d frames" % self.saved, flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--world", required=True, help="gz world name")
    ap.add_argument("--out", required=True)
    ap.add_argument("--prefix", default="gz_drone")
    ap.add_argument("--period", type=float, default=6.0, help="sim seconds between frames")
    ap.add_argument("--dist", type=float, nargs="+", default=[3.0], help="camera distance(s) [m], cycled")
    ap.add_argument("--elev", type=float, default=18.0)
    ap.add_argument("--az", type=float, nargs="+", default=[150.0, 30.0, -140.0, -35.0, 90.0])
    ap.add_argument("--min-alt", type=float, default=0.8)
    ap.add_argument("--min-cam-z", type=float, default=-1e9)
    ap.add_argument("--max-frames", type=int, default=40)
    ap.add_argument("--timeout", type=float, default=1800.0)
    a, _ = ap.parse_known_args()
    Capture(a).run()


if __name__ == "__main__":
    main()
