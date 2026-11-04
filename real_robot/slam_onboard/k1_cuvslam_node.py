"""Run ON a K1 (humble, ~/k1_slam/venv with PyCuVSLAM 17.0.1+cu12). cuVSLAM RGB-D SLAM from the head camera.

Isaac ROS 3.2 visual_slam has no RGB-D mode and the K1 publishes no right-eye image, so this node drives the
cuVSLAM library directly and feeds nvblox:
  in : /boostercamera/head/rgb (nv12), /boostercamera/head/depth (32FC1 millimetres, aligned, same stamps),
       /boostercamera/head/rgb/camera_info, /imu/data, /odom (odom->trunk, same as k1-tf.service),
       TF trunk->head_color_optical_frame + ankles, read once at start (/tf runs at ~840 Hz; listening costs a core)
  out: TF map->odom; /{robot_ns}/slam/odometry, /{robot_ns}/slam/path;
       debug for RViz: /{robot_ns}/slam/landmarks/{local,global} (PointCloud2, map; global = loop-closed SLAM map), /{robot_ns}/slam/features/image/compressed
       (tracked features on the camera image), /{robot_ns}/slam/status (text Marker), /{robot_ns}/slam/depth_points
       (depth every 8th pixel as map-frame points, for the laptop 2-D map: k1_grid_map.py);
       /{robot_ns}/slam/depth/{image,camera_info} (16UC1 mm) and /{robot_ns}/slam/color/{image,camera_info} (rgb8) for nvblox.
The head moves: cuVSLAM tracks the camera (rig = camera) and the trunk pose is camera pose x the live head
kinematics (/joint_states head yaw/pitch at the image stamp, URDF joint origins). The IMU is in the trunk, so cuVSLAM's
inertial modes (fixed camera-IMU rig) do not apply; the IMU only levels the map and the published pose.
The map frame sits on the floor under the start pose, gravity-levelled from /imu/data, yaw = start heading.
With ground_lock (default) the published pose keeps x, y, yaw from SLAM, takes roll/pitch from the IMU (gravity) and
holds the trunk at its start height above the floor: RGB-D drift in z/tilt never puts the robot under the floor.
"""
import collections
import struct
import threading
import time

import cuvslam
import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from builtin_interfaces.msg import Time as TimeMsg
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, JointState, PointCloud2, PointField
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

CAM_QOS = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)


def stamp_ns(h):
    return h.stamp.sec * 10**9 + h.stamp.nanosec


# Images are taken and given as serialized CDR (raw=True subscriptions, bytes publish): building rclpy Image
# objects for ~40 MB/s of camera data saturated 4 Orin NX cores. Little-endian CDR, alignment counted after the
# 4-byte encapsulation header.
def _align(o):
    return 4 + ((o - 4 + 3) & ~3)


def _get_str(b, o):
    o = _align(o)
    n = struct.unpack_from('<I', b, o)[0]
    return bytes(b[o + 4:o + 3 + n]).decode(), o + 4 + n


def parse_image(b):
    """Serialized sensor_msgs/Image -> (header, height, width, encoding, zero-copy data view)."""
    sec, nsec = struct.unpack_from('<iI', b, 4)
    frame_id, o = _get_str(b, 12)
    o = _align(o)
    h, w = struct.unpack_from('<II', b, o)
    enc, o = _get_str(b, o + 8)
    o = _align(o + 1)  # is_bigendian
    n = struct.unpack_from('<I', b, o + 4)[0]
    return Header(stamp=TimeMsg(sec=sec, nanosec=nsec), frame_id=frame_id), h, w, enc, memoryview(b)[o + 8:o + 8 + n]


def _put_str(b, s):
    b += b'\0' * (_align(len(b)) - len(b))
    e = s.encode() + b'\0'
    b += struct.pack('<I', len(e)) + e


def image_cdr(header, h, w, enc, step, data):
    b = bytearray(b'\x00\x01\x00\x00')
    b += struct.pack('<iI', header.stamp.sec, header.stamp.nanosec)
    _put_str(b, header.frame_id)
    b += b'\0' * (_align(len(b)) - len(b))
    b += struct.pack('<II', h, w)
    _put_str(b, enc)
    b += b'\0'  # is_bigendian
    b += b'\0' * (_align(len(b)) - len(b))
    data = memoryview(data).cast('B')  # len() of a 2-D numpy view counts rows, not bytes
    b += struct.pack('<II', step, len(data))
    b += data
    return bytes(b)


def quat_to_mat(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(r):
    w = np.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    return x, y, z, w


XYZ = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]


def cloud(header, pts):
    """float32 (N, 3) -> PointCloud2 (sensor_msgs_py packs point by point: too slow for the global map)."""
    return PointCloud2(header=header, height=1, width=len(pts), fields=XYZ, is_bigendian=False, point_step=12,
                       row_step=12 * len(pts), is_dense=True, data=pts.tobytes())


def rot_z(a):
    return np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])


def rot_y(a):
    return np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])


# real_robot/urdf/k1_robot_description.urdf: trunk -> aahead_yaw (z) -> aahead_pitch (y) -> camera (fixed)
HEAD_YAW_XYZ, HEAD_PITCH_XYZ = (0.0056, 0.0, 0.2149), (0.0, 0.0, 0.033)


def trunk_T_pitch_link(yaw, pitch):
    a, b = np.eye(4), np.eye(4)
    a[:3, :3], a[:3, 3] = rot_z(yaw), HEAD_YAW_XYZ
    b[:3, :3], b[:3, 3] = rot_y(pitch), HEAD_PITCH_XYZ
    return a @ b


def make_T(t, q):
    T = np.eye(4)
    T[:3, :3] = quat_to_mat(*q)
    T[:3, 3] = t
    return T


def tf_to_T(tf):
    tr, ro = tf.transform.translation, tf.transform.rotation
    return make_T((tr.x, tr.y, tr.z), (ro.x, ro.y, ro.z, ro.w))


class K1CuVSLAM(Node):
    def __init__(self):
        super().__init__('k1_cuvslam')
        self.declare_parameter('robot_ns', 'k1_0')
        self.declare_parameter('use_slam_pose', True)    # loop-closed SLAM pose; False = pure visual odometry
        self.declare_parameter('color_hz', 5.0)          # nvblox only colours the mesh at 5 Hz
        self.declare_parameter('depth_hz', 5.0)          # depth to nvblox; every frame is still tracked
        self.declare_parameter('ankle_to_floor', 0.035)  # ankle_roll_link origin height above the sole [m]
        self.declare_parameter('ground_lock', True)      # flat floor: z fixed, roll/pitch from the IMU
        self.declare_parameter('debug_hz', 5.0)          # features image + local landmarks; global map + path every 2 s
        ns = self.get_parameter('robot_ns').value
        self.use_slam = self.get_parameter('use_slam_pose').value
        self.color_dt = 1e9 / self.get_parameter('color_hz').value
        self.depth_dt = 1e9 / self.get_parameter('depth_hz').value
        self.ankle_to_floor = self.get_parameter('ankle_to_floor').value
        self.ground_lock = self.get_parameter('ground_lock').value
        self.debug_dt = 1e9 / self.get_parameter('debug_hz').value
        p = f'/{ns}/slam'

        # TF is read only at start, on a helper node with its own executor (spin_thread=True would spin this whole
        # node in a second executor too); both are torn down once the tracker exists.
        self.tf_buf = Buffer()
        self.tf_node = rclpy.create_node('k1_cuvslam_tf')
        TransformListener(self.tf_buf, self.tf_node)
        self.tf_ex = SingleThreadedExecutor()
        self.tf_ex.add_node(self.tf_node)
        threading.Thread(target=self.tf_ex.spin, daemon=True).start()
        self.tf_br = TransformBroadcaster(self)
        self.pub_odom = self.create_publisher(Odometry, f'{p}/odometry', 10)
        self.pub_path = self.create_publisher(Path, f'{p}/path', 1)
        self.pub_depth = self.create_publisher(Image, f'{p}/depth/image', 5)
        self.pub_depth_info = self.create_publisher(CameraInfo, f'{p}/depth/camera_info', 5)
        self.pub_color = self.create_publisher(Image, f'{p}/color/image', 5)
        self.pub_color_info = self.create_publisher(CameraInfo, f'{p}/color/camera_info', 5)
        self.pub_local = self.create_publisher(PointCloud2, f'{p}/landmarks/local', 2)
        self.pub_global = self.create_publisher(PointCloud2, f'{p}/landmarks/global', 1)
        self.pub_feat = self.create_publisher(CompressedImage, f'{p}/features/image/compressed', 2)
        self.pub_status = self.create_publisher(Marker, f'{p}/status', 2)
        self.pub_points = self.create_publisher(PointCloud2, f'{p}/depth_points', 2)
        self.last_debug = self.last_global = 0
        self.n_global = 0
        self.path = Path()
        self.path.header.frame_id = 'map'

        self.info = None
        self.imu_q = None
        self.rgb = {}  # stamp_ns -> nv12 Image
        self.tracker = None
        self.map_T_world = None
        self.last_color = self.last_depth = 0
        self.n = self.lost = self.unpaired = 0
        self.n_rgb = self.n_depth = 0
        self.t_track = 0.0
        # Callbacks only stash messages; tracking runs in a worker thread on the newest RGB-D pair so a slow
        # track() call drops frames instead of starving the subscriptions.
        self.lock = threading.Lock()
        self.new_depth = threading.Event()
        self.depth = None
        cb = ReentrantCallbackGroup()
        self.create_subscription(CameraInfo, '/boostercamera/head/rgb/camera_info', self.on_info, CAM_QOS, callback_group=cb)
        self.create_subscription(Imu, '/imu/data', self.on_imu, QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=cb)
        self.create_subscription(Image, '/boostercamera/head/rgb', self.on_rgb, CAM_QOS, callback_group=cb, raw=True)
        self.create_subscription(Image, '/boostercamera/head/depth', self.on_depth, CAM_QOS, callback_group=cb, raw=True)
        self.odom_raw = None
        self.js = collections.deque(maxlen=200)  # (stamp_ns, raw) ~0.5 s of /joint_states
        self.create_subscription(JointState, '/joint_states', self.on_js, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=cb, raw=True)
        self.create_subscription(Odometry, '/odom', self.on_odom, QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=cb, raw=True)
        self.create_timer(10.0, self.report, callback_group=cb)
        threading.Thread(target=self.worker, daemon=True).start()

    def on_info(self, m):
        self.info = m

    def on_imu(self, m):
        o = m.orientation
        self.imu_q = (o.x, o.y, o.z, o.w)

    def on_rgb(self, raw):
        self.n_rgb += 1
        m = parse_image(raw)
        with self.lock:
            self.rgb[stamp_ns(m[0])] = m
            if len(self.rgb) > 10:
                del self.rgb[min(self.rgb)]

    def on_odom(self, raw):
        self.odom_raw = raw  # ~380 Hz: keep the bytes, deserialize only the one a frame uses

    def on_js(self, raw):
        sec, nsec = struct.unpack_from('<iI', raw, 4)
        self.js.append((sec * 10**9 + nsec, raw))

    def head_at(self, t):
        """(yaw, pitch) from the /joint_states sample closest to stamp t (ns)."""
        js = list(self.js)
        if not js:
            return None
        _, raw = min(js, key=lambda e: abs(e[0] - t))
        p = deserialize_message(raw, JointState).position  # joints 0, 1 = aahead_yaw_joint, aahead_pitch_joint
        return p[0], p[1]

    def on_depth(self, raw):
        self.n_depth += 1
        with self.lock:
            self.depth = raw
        self.new_depth.set()

    def worker(self):
        while rclpy.ok():
            if not self.new_depth.wait(0.5):
                continue
            self.new_depth.clear()
            with self.lock:
                raw, self.depth = self.depth, None
            if raw is None:
                continue
            m = parse_image(raw)
            t, rgb = stamp_ns(m[0]), None
            for _ in range(5):  # the matching RGB frame can land a few ms after the depth frame
                with self.lock:
                    rgb = self.rgb.pop(t, None)
                if rgb is not None:
                    break
                time.sleep(0.005)
            if rgb is None:
                self.unpaired += 1
                continue
            self.process(m, rgb)

    def lookup(self, parent, child, t=None):
        try:
            return tf_to_T(self.tf_buf.lookup_transform(parent, child, t or Time()))
        except Exception:
            return None

    def init_tracker(self, frame_id, t):
        rig_T_cam = self.lookup('trunk', frame_id)
        head = self.head_at(t)
        l_ankle = self.lookup('trunk', 'left_ankle_roll_link')
        r_ankle = self.lookup('trunk', 'right_ankle_roll_link')
        if rig_T_cam is None or l_ankle is None or r_ankle is None or self.imu_q is None or head is None:
            return False
        cam = cuvslam.Camera()
        cam.size = (self.info.width, self.info.height)
        cam.focal = (self.info.k[0], self.info.k[4])
        cam.principal = (self.info.k[2], self.info.k[5])
        cam.distortion = cuvslam.Distortion(cuvslam.Distortion.Model.Pinhole)  # topics are rectified (D empty)
        cam.rig_from_camera = cuvslam.Pose(rotation=[0.0, 0.0, 0.0, 1.0], translation=[0.0, 0.0, 0.0])  # rig = camera
        self.pitch_T_cam = np.linalg.inv(trunk_T_pitch_link(*head)) @ rig_T_cam  # fixed camera mount, from the live TF
        rig = cuvslam.Rig()
        rig.cameras = [cam]
        rgbd = cuvslam.Tracker.OdometryRGBDSettings()
        rgbd.depth_scale_factor = 1000.0  # depth in millimetres
        rgbd.depth_camera_id = 0
        cfg = cuvslam.Tracker.OdometryConfig()
        cfg.odometry_mode = cuvslam.Tracker.OdometryMode.RGBD
        cfg.rgbd_settings = rgbd
        cfg.enable_landmarks_export = True       # local map (landmarks seen in the last frame)
        cfg.enable_observations_export = True    # 2-D features for the debug image
        slam_cfg = cuvslam.Tracker.SlamConfig()
        slam_cfg.enable_reading_internals = True  # global map = SLAM map landmarks, corrected by loop closures
        self.tracker = cuvslam.Tracker(rig, cfg, slam_cfg if self.use_slam else None)
        k = self.info.k
        v, u = np.mgrid[0:self.info.height:8, 0:self.info.width:8]
        self.ray_x, self.ray_y = ((u - k[2]) / k[0]).astype(np.float32), ((v - k[5]) / k[4]).astype(np.float32)
        self.tf_ex.shutdown()  # the camera mount and leg geometry are only needed once
        self.tf_node.destroy_node()

        # cuVSLAM world = trunk at start. map = same origin dropped to the floor, levelled by the IMU.
        R = quat_to_mat(*self.imu_q)
        yaw = np.arctan2(R[1, 0], R[0, 0])
        h0 = -min(l_ankle[2, 3], r_ankle[2, 3]) + self.ankle_to_floor
        self.h0 = h0
        map_T_trunk0 = np.eye(4)
        map_T_trunk0[:3, :3] = rot_z(-yaw) @ R  # roll/pitch of the start trunk, yaw removed
        map_T_trunk0[2, 3] = h0
        self.map_T_world = map_T_trunk0 @ rig_T_cam  # cuVSLAM world = start camera pose
        self.get_logger().info(f'tracker ready: trunk height {h0:.3f} m, cam at {np.round(rig_T_cam[:3, 3], 3)} in trunk, '
                               f'{np.round(self.pitch_T_cam[:3, 3], 3)} in head pitch link (URDF 0.060 0.035 0.093), head {np.round(head, 3)}')
        return True

    def process(self, m, rgb):
        header, h, w, enc, data = m
        t = stamp_ns(header)
        if self.info is None:
            return
        if enc != '32FC1' or rgb[3] != 'nv12':
            self.get_logger().error(f'unexpected encodings depth={enc} rgb={rgb[3]}', throttle_duration_sec=5)
            return
        if self.tracker is None and not self.init_tracker(header.frame_id, t):
            return
        d = np.frombuffer(data, np.float32).reshape(h, w)
        d16 = np.clip(np.nan_to_num(d, nan=0.0, posinf=0.0), 0, 65535).astype(np.uint16)
        y = np.frombuffer(rgb[4], np.uint8)[: h * w].reshape(h, w)

        t0 = time.time()
        try:
            odom, slam = self.tracker.track(t, images=[y], depths=[d16])
        except ValueError as e:
            self.get_logger().warn(f'track: {e}')
            return
        self.t_track += time.time() - t0
        self.n += 1
        pose = slam if (self.use_slam and slam is not None) else (odom.world_from_rig.pose if odom.world_from_rig else None)
        if pose is None:
            self.lost += 1
            return
        head = self.head_at(t)
        trunk_T_cam = trunk_T_pitch_link(*head) @ self.pitch_T_cam
        map_T_trunk = self.map_T_world @ make_T(pose.translation, pose.rotation) @ np.linalg.inv(trunk_T_cam)
        if self.ground_lock:
            R = quat_to_mat(*self.imu_q)
            yaw_slam = np.arctan2(map_T_trunk[1, 0], map_T_trunk[0, 0])
            map_T_trunk[:3, :3] = rot_z(yaw_slam - np.arctan2(R[1, 0], R[0, 0])) @ R
            map_T_trunk[2, 3] = self.h0
        self.publish_pose(header, map_T_trunk)
        if t - self.last_debug >= self.debug_dt:
            self.last_debug = t
            self.publish_debug(header, rgb[4], h, w, map_T_trunk, t)
            self.publish_points(header, d[::8, ::8], map_T_trunk @ trunk_T_cam)

        if t - self.last_depth >= self.depth_dt and self.pub_depth.get_subscription_count():
            self.last_depth = t
            self.pub_depth.publish(image_cdr(header, h, w, '16UC1', w * 2, d16.data))
            self.pub_depth_info.publish(self.info_at(header))
        if t - self.last_color >= self.color_dt and self.pub_color.get_subscription_count():
            self.last_color = t
            nv12 = np.frombuffer(rgb[4], np.uint8)[: h * w * 3 // 2].reshape(h * 3 // 2, w)
            c = cv2.cvtColor(nv12, cv2.COLOR_YUV2RGB_NV12)
            self.pub_color.publish(image_cdr(header, h, w, 'rgb8', w * 3, c.data))
            self.pub_color_info.publish(self.info_at(header))

    def to_map(self, pts):
        pts = np.asarray(pts, np.float32).reshape(-1, 3)
        return (pts @ self.map_T_world[:3, :3].T.astype(np.float32) + self.map_T_world[:3, 3].astype(np.float32)).astype(np.float32)

    def publish_debug(self, header, nv12, h, w, map_T_trunk, t):
        mh = Header(stamp=header.stamp, frame_id='map')
        lms = self.tracker.get_last_landmarks()
        self.pub_local.publish(cloud(mh, self.to_map([l.coords for l in lms])))
        if self.use_slam and t - self.last_global >= 2e9:
            self.last_global = t
            g = self.tracker.get_slam_landmarks(cuvslam.Tracker.SlamDataLayer.Map)
            pts = self.to_map([l.coords for l in g.landmarks] if g else [])
            self.n_global = len(pts)
            self.pub_global.publish(cloud(mh, pts))
            self.publish_path(mh)
        if self.pub_feat.get_subscription_count():
            nv12 = np.frombuffer(nv12, np.uint8)[: h * w * 3 // 2].reshape(h * 3 // 2, w)
            img = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)[::2, ::2].copy()
            for ob in self.tracker.get_last_observations(0):
                cv2.circle(img, (int(ob.u / 2), int(ob.v / 2)), 2, (0, 255, 0), -1)
            ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                self.pub_feat.publish(CompressedImage(header=header, format='jpeg', data=jpg.tobytes()))
        m = Marker(header=mh, ns='slam', id=0, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map_T_trunk[0, 3], map_T_trunk[1, 3], map_T_trunk[2, 3] + 0.6
        m.pose.orientation.w = 1.0
        m.scale.z = 0.12
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
        m.text = f'local {len(lms)}  global {self.n_global}  lost {self.lost}'
        self.pub_status.publish(m)

    def publish_points(self, header, d, T):
        z = d / 1000.0
        ok = (z > 0.2) & (z < 4.0)
        pts = np.stack([self.ray_x[ok] * z[ok], self.ray_y[ok] * z[ok], z[ok]], 1)
        pts = (pts @ T[:3, :3].T + T[:3, 3]).astype(np.float32)
        self.pub_points.publish(cloud(Header(stamp=header.stamp, frame_id='map'), pts))

    def publish_path(self, mh):
        """Whole trajectory re-solved on the current pose graph: no jumps where loop closures snapped the live pose."""
        poses = self.tracker.get_all_slam_poses()[::5]
        xyz = self.to_map([p.pose.translation for p in poses])
        if self.ground_lock:
            xyz[:, 2] = self.h0
        self.path.header = mh
        self.path.poses = []
        for x, y, z in xyz:
            ps = PoseStamped(header=mh)
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(x), float(y), float(z)
            ps.pose.orientation.w = 1.0
            self.path.poses.append(ps)
        self.pub_path.publish(self.path)

    def info_at(self, header):
        i = CameraInfo()
        i.header, i.width, i.height, i.k, i.p, i.r = header, self.info.width, self.info.height, self.info.k, self.info.p, self.info.r
        i.distortion_model, i.d = 'plumb_bob', [0.0] * 5
        return i

    def publish_pose(self, header, map_T_trunk):
        odom_T_trunk = None
        if self.odom_raw is not None:
            p = deserialize_message(self.odom_raw, Odometry).pose.pose
            odom_T_trunk = make_T((p.position.x, p.position.y, p.position.z),
                                  (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
        if odom_T_trunk is not None:
            T = map_T_trunk @ np.linalg.inv(odom_T_trunk)
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = header.stamp, 'map', 'odom'
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = T[:3, 3]
            r = tf.transform.rotation
            r.x, r.y, r.z, r.w = mat_to_quat(T[:3, :3])
            self.tf_br.sendTransform(tf)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = header.stamp, 'map', 'trunk'
        pp = o.pose.pose
        pp.position.x, pp.position.y, pp.position.z = map_T_trunk[:3, 3]
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = mat_to_quat(map_T_trunk[:3, :3])
        self.pub_odom.publish(o)
        if not self.use_slam and self.n % 5 == 0:  # with SLAM, publish_path redraws the corrected trajectory
            ps = PoseStamped(header=o.header, pose=pp)
            self.path.header.stamp = header.stamp
            self.path.poses.append(ps)
            self.pub_path.publish(self.path)

    def report(self):
        ms = 1e3 * self.t_track / max(self.n, 1)
        self.get_logger().info(f'tracked {self.n / 10:.1f} Hz, {ms:.1f} ms/frame, lost {self.lost}, unpaired {self.unpaired}, in rgb {self.n_rgb / 10:.1f} Hz depth {self.n_depth / 10:.1f} Hz')
        self.n = self.lost = self.unpaired = self.n_rgb = self.n_depth = 0
        self.t_track = 0.0


def main():
    rclpy.init()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(K1CuVSLAM())
    ex.spin()


if __name__ == '__main__':
    main()
