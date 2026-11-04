"""Laptop: 2-D occupancy map, costmap and collision marker from the K1 SLAM stream (stand-in until nvblox).

in : /{ns}/slam/depth_points (map-frame depth samples from k1_cuvslam_node), /{ns}/slam/odometry (trunk pose)
out: /{ns}/map (OccupancyGrid), /{ns}/costmap (inflated, 0-100), /{ns}/collision (footprint Marker, red when an
     obstacle is inside collision_radius; text = distance to the nearest obstacle)
Points between obstacle_z_min and obstacle_z_max above the floor are obstacles; every ray from the robot to a point
clears the cells it crosses (log-odds).
  python3 k1_grid_map.py --ros-args -p robot_ns:=k1_0
"""
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
L_OCC, L_FREE, L_MIN, L_MAX = 0.85, -0.4, -2.0, 3.5


class GridMap(Node):
    def __init__(self):
        super().__init__('k1_grid_map')
        for name, val in (('robot_ns', 'k1_0'), ('resolution', 0.05), ('size_m', 40.0), ('obstacle_z_min', 0.08),
                          ('obstacle_z_max', 1.2), ('robot_radius', 0.2), ('inflation_radius', 0.6),
                          ('collision_radius', 0.35), ('max_range', 3.5)):
            self.declare_parameter(name, val)
        g = lambda n: self.get_parameter(n).value
        ns = g('robot_ns')
        self.res, self.zmin, self.zmax, self.max_range = g('resolution'), g('obstacle_z_min'), g('obstacle_z_max'), g('max_range')
        self.r_robot, self.r_infl, self.r_coll = g('robot_radius'), g('inflation_radius'), g('collision_radius')
        self.n = int(g('size_m') / self.res)
        self.origin = -self.n * self.res / 2  # map centred on the SLAM start
        self.logodds = np.zeros((self.n, self.n), np.float32)  # [row = y, col = x]
        self.robot = None
        self.pub_map = self.create_publisher(OccupancyGrid, f'/{ns}/map', LATCHED)
        self.pub_cost = self.create_publisher(OccupancyGrid, f'/{ns}/costmap', LATCHED)
        self.pub_coll = self.create_publisher(Marker, f'/{ns}/collision', 2)
        self.create_subscription(Odometry, f'/{ns}/slam/odometry', self.on_odom, 5)
        self.create_subscription(PointCloud2, f'/{ns}/slam/depth_points', self.on_points, 2)
        self.create_timer(1.0, self.publish_maps)
        self.create_timer(0.2, self.publish_collision)
        self.dist = None  # metres to the nearest obstacle per cell

    def on_odom(self, m):
        p = m.pose.pose.position
        self.robot = (p.x, p.y, p.z)

    def cell(self, xy):
        return np.floor((xy - self.origin) / self.res).astype(np.int32)

    def on_points(self, m):
        if self.robot is None or m.width == 0:
            return
        pts = np.frombuffer(m.data, np.float32).reshape(-1, 3)
        o = np.array(self.robot[:2], np.float32)
        d = pts[:, :2] - o
        rng = np.linalg.norm(d, axis=1)
        keep = (rng < self.max_range) & (pts[:, 2] < self.zmax)
        pts, d, rng = pts[keep], d[keep], rng[keep]
        occ = pts[:, 2] > self.zmin
        # free space: sample every ray at grid resolution, stopping one cell short of the end point
        k = int(self.max_range / self.res)
        t = np.arange(k, dtype=np.float32)[None, :] * self.res
        valid = t < (rng[:, None] - self.res)
        u = d / np.maximum(rng, 1e-6)[:, None]
        free_xy = o + u[:, None, :] * t[..., None]
        fc = self.cell(free_xy[valid])
        fc = np.unique(fc[:, 1] * self.n + fc[:, 0]) if len(fc) else fc
        oc = self.cell(pts[occ, :2])
        oc = np.unique(oc[:, 1] * self.n + oc[:, 0]) if len(oc) else oc
        flat = self.logodds.reshape(-1)
        for idx, val in ((fc, L_FREE), (oc, L_OCC)):
            idx = idx[(idx >= 0) & (idx < flat.size)]
            flat[idx] = np.clip(flat[idx] + val, L_MIN, L_MAX)

    def grid_msg(self, data):
        g = OccupancyGrid()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.info.resolution, g.info.width, g.info.height = self.res, self.n, self.n
        g.info.origin.position.x = g.info.origin.position.y = self.origin
        g.info.origin.orientation.w = 1.0
        g.data = data.reshape(-1).tolist()
        return g

    def publish_maps(self):
        occ = self.logodds > 0.5
        grid = np.full(self.logodds.shape, -1, np.int8)
        grid[self.logodds < -0.5] = 0
        grid[occ] = 100
        self.pub_map.publish(self.grid_msg(grid))
        self.dist = distance_transform_edt(~occ) * self.res if occ.any() else np.full(occ.shape, np.inf, np.float32)
        # costmap: 100 inside the robot radius of an obstacle, linear fall-off to 0 at the inflation radius
        cost = np.clip((self.r_infl - self.dist) / (self.r_infl - self.r_robot), 0, 1) * 99
        cost = cost.astype(np.int8)
        cost[self.dist <= self.r_robot] = 100
        cost[grid == -1] = -1
        self.pub_cost.publish(self.grid_msg(cost))

    def publish_collision(self):
        if self.robot is None or self.dist is None:
            return
        c = self.cell(np.array(self.robot[:2]))
        dmin = float(self.dist[c[1], c[0]]) if 0 <= c[0] < self.n and 0 <= c[1] < self.n else np.inf
        hit = dmin < self.r_coll
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type = 'collision', 0, Marker.CYLINDER
        m.pose.position.x, m.pose.position.y, m.pose.position.z = self.robot[0], self.robot[1], 0.01
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 2 * self.r_coll
        m.scale.z = 0.02
        m.color.r, m.color.g, m.color.b, m.color.a = (1.0, 0.0, 0.0, 0.6) if hit else (0.0, 1.0, 0.0, 0.3)
        self.pub_coll.publish(m)
        t = Marker()
        t.header = m.header
        t.ns, t.id, t.type = 'collision', 1, Marker.TEXT_VIEW_FACING
        t.pose.position.x, t.pose.position.y, t.pose.position.z = self.robot[0], self.robot[1], 1.3
        t.pose.orientation.w = 1.0
        t.scale.z = 0.12
        t.color.r, t.color.g, t.color.b, t.color.a = (1.0, 0.3, 0.3, 1.0) if hit else (1.0, 1.0, 1.0, 1.0)
        t.text = f'obstacle {dmin:.2f} m' + (' COLLISION' if hit else '') if np.isfinite(dmin) else 'obstacle -'
        self.pub_coll.publish(t)


def main():
    rclpy.init()
    rclpy.spin(GridMap())


if __name__ == '__main__':
    main()
