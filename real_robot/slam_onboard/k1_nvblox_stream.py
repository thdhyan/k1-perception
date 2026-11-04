"""Run ON a K1 next to run_slam.sh: streams nvblox inputs to the laptop (k1_nvblox_rx.py) over TCP 7448.

No DDS crosses Wi-Fi (see k1_slam_relay.py). Subscribing here makes k1_cuvslam_node publish its 5 Hz depth
(16UC1 mm, zlib) and colour (rgb8, sent as JPEG) for tracked frames. Pose and the robot model come from the SLAM
relay stream (laptop_rviz.sh); this adds the camera driver's static TF to the head camera, which the repo URDF lacks.
Wire: one JSON line [[topic, type, transient_local, codec], ...], then frames <u8 topic index><u32 length><payload>;
codec raw = CDR, zlib = zlib(CDR), jpeg = CDR of a CompressedImage the laptop turns back into an rgb8 Image.
  python3 k1_nvblox_stream.py [robot_ns]
"""
import json
import queue
import socket
import struct
import sys
import threading
import zlib

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from tf2_msgs.msg import TFMessage

PORT = 7448
QOS = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
LATCHED = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
CAM_FRAMES = ('head_color_optical_frame', 'head_depth_optical_frame')  # kept from /tf_static, plus their parents


def main():
    ns = sys.argv[1] if len(sys.argv) > 1 else 'k1_0'
    p = f'/{ns}/slam'
    topics = [(f'{p}/depth/image', 'sensor_msgs/msg/Image', False, 'zlib'),
              (f'{p}/depth/camera_info', 'sensor_msgs/msg/CameraInfo', False, 'raw'),
              (f'{p}/color/image', 'sensor_msgs/msg/Image', False, 'jpeg'),
              (f'{p}/color/camera_info', 'sensor_msgs/msg/CameraInfo', False, 'raw'),
              ('/tf_static', 'tf2_msgs/msg/TFMessage', True, 'raw')]
    idx = {t[0]: i for i, t in enumerate(topics)}
    hello = (json.dumps(topics) + '\n').encode()
    q = queue.Queue(maxsize=10)  # ~1 s of frames; a lagging laptop loses frames, nothing grows
    static = {}  # child frame -> TransformStamped, resent to every new client

    def send(topic, payload):
        try:
            q.put_nowait(struct.pack('<BI', idx[topic], len(payload)) + payload)
        except queue.Full:
            pass

    def serve():
        srv = socket.socket()
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('0.0.0.0', PORT))
        srv.listen(1)
        while True:
            c, addr = srv.accept()
            c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            c.settimeout(5.0)
            print(f'client {addr[0]}', flush=True)
            while not q.empty():
                q.get_nowait()
            try:
                c.sendall(hello)
                if static:
                    tf = serialize_message(TFMessage(transforms=list(static.values())))
                    c.sendall(struct.pack('<BI', idx['/tf_static'], len(tf)) + tf)
                while True:
                    c.sendall(q.get())
            except OSError as e:
                print(f'client gone: {e}', flush=True)
                c.close()

    threading.Thread(target=serve, daemon=True).start()

    rclpy.init()
    node = Node('k1_nvblox_stream')
    log = node.get_logger()

    def on_depth(raw):
        send(f'{p}/depth/image', zlib.compress(bytes(raw), 1))

    def on_color(raw):
        m = deserialize_message(raw, Image)
        rgb = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        ok, jpg = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 85])
        if ok:
            send(f'{p}/color/image', serialize_message(CompressedImage(header=m.header, format='jpeg', data=jpg.tobytes())))

    def on_static(m):
        new = False
        for t in m.transforms:
            if t.child_frame_id in CAM_FRAMES and t.child_frame_id not in static:
                static[t.child_frame_id] = t
                new = True
        if new:
            log.info(f'camera static TF: {[(t.header.frame_id, t.child_frame_id) for t in static.values()]}')
            send('/tf_static', serialize_message(TFMessage(transforms=list(static.values()))))

    node.create_subscription(Image, f'{p}/depth/image', on_depth, QOS, raw=True)
    node.create_subscription(Image, f'{p}/color/image', on_color, QOS, raw=True)
    for side in ('depth', 'color'):
        topic = f'{p}/{side}/camera_info'
        node.create_subscription(CameraInfo, topic, lambda raw, t=topic: send(t, bytes(raw)), QOS, raw=True)
    node.create_subscription(TFMessage, '/tf_static', on_static, LATCHED)
    log.info(f'streaming nvblox inputs on tcp :{PORT}')
    rclpy.spin(node)


if __name__ == '__main__':
    main()
