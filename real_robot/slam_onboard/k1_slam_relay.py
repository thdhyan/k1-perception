"""Run ON a K1 next to run_slam.sh: streams the SLAM visualisation topics to the laptop (k1_slam_rx.py) over TCP.

No DDS crosses Wi-Fi: Humble FastDDS on the K1 talking to the laptop's Jazzy FastDDS grew whichever K1 process
the laptop read from by GBs until the OOM killer hit (k1-tf, SLAM, an earlier DDS relay). Messages are sent as
serialized CDR, which is the same on both distros for these types.

Instead of /tf (~840 Hz) the laptop gets map->trunk from the SLAM pose and /joint_states at 30 Hz for its own
robot_state_publisher (laptop_rviz.sh). One client at a time; frames are dropped when it lags.
Wire: one JSON line [[topic, type, transient_local], ...], then frames <u8 topic index><u32 length><CDR>.
"""
import json
import queue
import socket
import struct
import sys
import threading

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from tf2_msgs.msg import TFMessage

PORT = 7447
QOS = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
BE = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)


def main():
    ns = sys.argv[1] if len(sys.argv) > 1 else 'k1_0'
    p = f'/{ns}/slam'
    # (topic, type, transient_local on the laptop)
    topics = [(f'{p}/odometry', 'nav_msgs/msg/Odometry', False),
              (f'{p}/path', 'nav_msgs/msg/Path', False),
              ('/tf', 'tf2_msgs/msg/TFMessage', False),
              ('/joint_states', 'sensor_msgs/msg/JointState', False),
              (f'{p}/landmarks/local', 'sensor_msgs/msg/PointCloud2', False),
              (f'{p}/landmarks/global', 'sensor_msgs/msg/PointCloud2', True),
              (f'{p}/features/image/compressed', 'sensor_msgs/msg/CompressedImage', False),
              (f'{p}/status', 'visualization_msgs/msg/Marker', False),
              (f'{p}/depth_points', 'sensor_msgs/msg/PointCloud2', False)]
    idx = {t[0]: i for i, t in enumerate(topics)}
    hello = (json.dumps(topics) + '\n').encode()

    q = queue.Queue(maxsize=30)  # a lagging laptop loses frames here; nothing grows

    def send(topic, raw):
        try:
            q.put_nowait(struct.pack('<BI', idx[topic], len(raw)) + bytes(raw))
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
                while True:
                    c.sendall(q.get())
            except OSError as e:
                print(f'client gone: {e}', flush=True)
                c.close()

    threading.Thread(target=serve, daemon=True).start()

    rclpy.init()
    node = Node('k1_slam_relay')

    def plain(topic):
        return lambda raw: send(topic, raw)

    for topic, typ, _ in topics:
        if topic in ('/tf', '/joint_states', f'{p}/odometry'):
            continue
        mod, _, name = typ.split('/')
        cls = getattr(__import__(f'{mod}.msg', fromlist=[name]), name)
        node.create_subscription(cls, topic, plain(topic), QOS, raw=True)

    def on_odom(raw):  # ~13 Hz
        send(f'{p}/odometry', raw)
        o = deserialize_message(raw, Odometry)
        tf = TransformStamped()
        tf.header, tf.child_frame_id = o.header, o.child_frame_id
        q = o.pose.pose
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = q.position.x, q.position.y, q.position.z
        tf.transform.rotation = q.orientation
        send('/tf', serialize_message(TFMessage(transforms=[tf])))

    node.create_subscription(Odometry, f'{p}/odometry', on_odom, QOS, raw=True)

    from sensor_msgs.msg import JointState
    js_n = [0]

    def on_js(raw):  # ~400 Hz in, every 13th out (~30 Hz)
        js_n[0] += 1
        if js_n[0] % 13 == 0:
            send('/joint_states', raw)

    node.create_subscription(JointState, '/joint_states', on_js, BE, raw=True)
    node.get_logger().info(f'streaming SLAM viz on tcp :{PORT}')
    rclpy.spin(node)


if __name__ == '__main__':
    main()
