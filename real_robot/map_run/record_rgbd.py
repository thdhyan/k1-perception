"""Run ON a K1 (humble). Record an RGB-D mapping session straight to disk (no rosbag).

  source /opt/ros/humble/setup.bash
  python3 record_rgbd.py --out ~/k1_maps/run1 --minutes 8

Writes:  images/cam0/NNNNNN.png   mono8 (Y plane of the nv12 RGB)
         depth/NNNNNN.png         uint16 millimetres, pixel-aligned with cam0
         frames.jsonl             {i, rgb_ns, depth_ns}
         odom.txt                 ns x y z qx qy qz qw  (/odom, every 5th msg)
         info.json                size + intrinsics from /boostercamera/head/rgb/camera_info
RGB and depth are paired by nearest header stamp (max 20 ms apart). Ctrl-C ends cleanly.
"""
import argparse, collections, json, os, time
import cv2, numpy as np, rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--minutes", type=float, default=8)
ap.add_argument("--max-pair-ms", type=float, default=20)
a = ap.parse_args()
out = os.path.expanduser(a.out)
for d in ("images/cam0", "depth"):
    os.makedirs(f"{out}/{d}", exist_ok=True)

ns = lambda h: h.stamp.sec * 10**9 + h.stamp.nanosec


class Rec(Node):
    def __init__(self):
        super().__init__("rgbd_recorder")
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.rgb = collections.deque(maxlen=12)  # (ns, Y plane)
        self.n = self.dropped = self.odom_n = 0
        self.stats = {"rgb": 0, "depth": 0}
        self.frames = open(f"{out}/frames.jsonl", "w")
        self.odom = open(f"{out}/odom.txt", "w")
        self.info = None
        self.create_subscription(Image, "/boostercamera/head/rgb", self.on_rgb, q)
        self.create_subscription(Image, "/boostercamera/head/depth", self.on_depth, q)
        self.create_subscription(CameraInfo, "/boostercamera/head/rgb/camera_info", self.on_info, q)
        self.create_subscription(Odometry, "/odom", self.on_odom,
                                 QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))

    def on_info(self, m):
        if self.info is None:
            self.info = {"width": m.width, "height": m.height, "fx": m.k[0], "fy": m.k[4], "cx": m.k[2], "cy": m.k[5]}
            json.dump(self.info, open(f"{out}/info.json", "w"))

    def on_rgb(self, m):
        self.stats["rgb"] += 1
        y = np.frombuffer(m.data, np.uint8)[: m.height * m.width].reshape(m.height, m.width).copy()  # nv12: Y first
        self.rgb.append((ns(m.header), y))

    def on_depth(self, m):
        self.stats["depth"] += 1
        if not self.rgb:
            return
        t = ns(m.header)
        rt, y = min(self.rgb, key=lambda r: abs(r[0] - t))
        if abs(rt - t) > a.max_pair_ms * 1e6:
            self.dropped += 1
            return
        d = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)
        d16 = np.clip(np.nan_to_num(d), 0, 65535).astype(np.uint16)  # topic is float32 millimetres
        cv2.imwrite(f"{out}/images/cam0/{self.n:06d}.png", y)
        cv2.imwrite(f"{out}/depth/{self.n:06d}.png", d16)
        self.frames.write(json.dumps({"i": self.n, "rgb_ns": rt, "depth_ns": t}) + "\n")
        self.n += 1

    def on_odom(self, m):
        self.odom_n += 1
        if self.odom_n % 5:
            return
        p, o = m.pose.pose.position, m.pose.pose.orientation
        self.odom.write(f"{ns(m.header)} {p.x} {p.y} {p.z} {o.x} {o.y} {o.z} {o.w}\n")


rclpy.init()
r = Rec()
end = time.time() + a.minutes * 60
last = time.time()
try:
    while time.time() < end:
        rclpy.spin_once(r, timeout_sec=0.1)
        if time.time() - last > 10:
            last = time.time()
            print(f"{time.strftime('%H:%M:%S')} saved={r.n} dropped_pairs={r.dropped} rgb_in={r.stats['rgb']} depth_in={r.stats['depth']}", flush=True)
except KeyboardInterrupt:
    pass
r.frames.close(); r.odom.close()
print(f"done: {r.n} RGB-D pairs, {r.dropped} unpaired, info={'ok' if r.info else 'MISSING'} -> {out}", flush=True)
