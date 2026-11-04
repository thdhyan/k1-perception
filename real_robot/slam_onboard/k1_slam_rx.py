"""Laptop: receive the K1 SLAM viz stream (k1_slam_relay.py on the robot, TCP 7447) and republish it on the local
ROS domain for RViz. Reconnects when the robot or the relay restarts.
  python3 k1_slam_rx.py [robot_ip]
"""
import json
import socket
import struct
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


def read_exact(f, n):
    b = f.read(n)
    if len(b) < n:
        raise ConnectionError('stream closed')
    return b


def main():
    robot = sys.argv[1] if len(sys.argv) > 1 else '192.168.50.82'
    rclpy.init()
    node = rclpy.create_node('k1_slam_rx')
    log = node.get_logger()
    pubs = {}  # topic -> publisher, kept across reconnects
    while rclpy.ok():
        try:
            s = socket.create_connection((robot, 7447), timeout=3)
            s.settimeout(10)
            f = s.makefile('rb')
            topics = json.loads(f.readline())
            out = []
            for topic, typ, latched in topics:
                if topic not in pubs:
                    qos = QoSProfile(depth=1 if latched else 5, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL if latched else DurabilityPolicy.VOLATILE)
                    pubs[topic] = node.create_publisher(get_message(typ), topic, qos)
                out.append(pubs[topic])
            log.info(f'connected to {robot}: {len(topics)} topics')
            while rclpy.ok():
                i, n = struct.unpack('<BI', read_exact(f, 5))
                out[i].publish(read_exact(f, n))
        except (OSError, ConnectionError, ValueError) as e:
            log.warn(f'{robot}:7447 {e}; retrying', throttle_duration_sec=10)
            time.sleep(1)


if __name__ == '__main__':
    main()
