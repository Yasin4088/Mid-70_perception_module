#!/usr/bin/env python3
"""假 Livox 数据源：验证 26 场地背景滤波效果（与 check_lidar_msg 严格一致：26 字节点/7 字段）。

4 个目标簇：
  A 开阔区  (7.0, 7.5, 0.7)  柱高 0.21  -> 应检出
  B 机关旁  (10.0, 7.5, 0.7) 柱高 2.92  -> 应被柱高滤波吃掉（能量机关死区）
  C 机关顶  (14.0, 7.5, 4.0) 柱高 3.54  -> 应检出（高于柱高）
  D 场角    (3.5, 2.5, 0.7)  柱高 5.11  -> 应被吃掉（场边结构）
话题 /pc_raw，frame livox_lidar_frame，10Hz。
"""
import math
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


def make_cloud(points, stamp_ns):
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='tag', offset=16, datatype=PointField.UINT8, count=1),
        PointField(name='resv', offset=17, datatype=PointField.UINT8, count=1),
        PointField(name='timestamp', offset=18, datatype=PointField.FLOAT64, count=1),
    ]
    buf = bytearray()
    for (x, y, z) in points:
        buf += struct.pack('<4fBBd', x, y, z, 100.0, 0, 0, stamp_ns / 1e9)
    msg = PointCloud2()
    msg.header.frame_id = 'livox_lidar_frame'
    msg.height = 1
    msg.width = len(points)
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 26
    msg.row_step = 26 * len(points)
    msg.is_dense = True
    msg.data = bytes(buf)
    return msg


def cluster(cx, cy, cz, n=50, spread=0.25, seed=0):
    import random
    rnd = random.Random(seed)
    pts = []
    for _ in range(n):
        pts.append((cx + rnd.gauss(0, spread), cy + rnd.gauss(0, spread), cz + rnd.gauss(0, spread * 0.4)))
    return pts


class FakeLivox(Node):
    def __init__(self):
        super().__init__('fake_livox')
        self.pub = self.create_publisher(PointCloud2, 'pc_raw', 10)
        self.timer = self.create_timer(0.1, self.cb)
        self.t0 = time.time()

    def cb(self):
        t = time.time() - self.t0
        # 小幅度运动保持跟踪
        ax = 7.0 + 0.4 * math.sin(t * 0.3)
        bx = 10.0 + 0.3 * math.sin(t * 0.25 + 1.0)
        cx = 14.0 + 0.3 * math.sin(t * 0.2 + 2.0)
        dx = 3.5 + 0.3 * math.sin(t * 0.35 + 3.0)
        pts = (
            cluster(ax, 7.5, 0.7, seed=1)
            + cluster(bx, 7.5, 0.7, seed=2)
            + cluster(cx, 7.5, 4.0, seed=3)
            + cluster(dx, 2.5, 0.7, seed=4)
        )
        msg = make_cloud(pts, self.get_clock().now().nanoseconds)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeLivox()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
