"""Laptop (inside the nvblox container): receive k1_nvblox_stream.py (K1, TCP 7448) and republish depth, colour,
camera_info and the head camera static TF on the local ROS domain for nvblox_node. Reconnects on drops.
  python3 k1_nvblox_rx.py [robot_ip]
"""
import json
import socket
import struct
import sys
import time
import zlib

import cv2
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CompressedImage, Image


def read_exact(f, n):
    b = f.read(n)
    if len(b) < n:
        raise ConnectionError('stream closed')
    return b


def jpeg_to_image(payload):
    c = deserialize_message(payload, CompressedImage)
    rgb = cv2.cvtColor(cv2.imdecode(np.frombuffer(c.data, np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
    return Image(header=c.header, height=rgb.shape[0], width=rgb.shape[1], encoding='rgb8',
                 step=rgb.shape[1] * 3, data=rgb.tobytes())


DECODE = {'raw': lambda b: b, 'zlib': zlib.decompress, 'jpeg': jpeg_to_image}


def main():
    robot = sys.argv[1] if len(sys.argv) > 1 else '192.168.50.82'
    rclpy.init()
    node = rclpy.create_node('k1_nvblox_rx')
    log = node.get_logger()
    pubs = {}
    while rclpy.ok():
        try:
            s = socket.create_connection((robot, 7448), timeout=3)
            s.settimeout(10)
            f = s.makefile('rb')
            topics = json.loads(f.readline())
            out = []
            for topic, typ, latched, codec in topics:
                if topic not in pubs:
                    qos = QoSProfile(depth=1 if latched else 5, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL if latched else DurabilityPolicy.VOLATILE)
                    pubs[topic] = node.create_publisher(get_message(typ), topic, qos)
                out.append((pubs[topic], DECODE[codec]))
            log.info(f'connected to {robot}:7448: {len(topics)} topics')
            while rclpy.ok():
                i, n = struct.unpack('<BI', read_exact(f, 5))
                pub, dec = out[i]
                pub.publish(dec(read_exact(f, n)))
        except (OSError, ConnectionError, ValueError, zlib.error) as e:
            log.warn(f'{robot}:7448 {e}; retrying', throttle_duration_sec=10)
            time.sleep(1)


if __name__ == '__main__':
    main()
