#!/usr/bin/env python3
# odometry.py — Amiga wheel odometry → ROS2 /odom
# Twist2d'den hız alır, pozisyonu dead reckoning ile entegre eder.

import argparse
import asyncio
import math
import csv
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from tf_transformations import quaternion_from_euler

from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file


class OdomPublisher(Node):
    def __init__(self, service_config: EventServiceConfig, log_file: Path):
        super().__init__("odometry_publisher")
        self.pub = self.create_publisher(Odometry, "/odom", 10)
        self.service_config = service_config
        self.log_file = log_file

        # Dead reckoning durumu
        self.x         = 0.0
        self.y         = 0.0
        self.yaw       = 0.0
        self.last_time = None   # saniye cinsinden float

        # CSV başlığı
        with open(self.log_file, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "time_sec", "time_nanosec",
                "linear_velocity_x", "angular_velocity",
                "pos_x", "pos_y", "yaw_rad"
            ])

    async def run(self):
        client = EventClient(self.service_config)
        subscription = self.service_config.subscriptions[0]

        self.get_logger().info(f"Listening to: {subscription.uri}")

        async for event, message in client.subscribe(subscription, decode=True):
            if not isinstance(message, Twist2d):
                self.get_logger().warn(f"Beklenmeyen mesaj tipi: {type(message)}")
                continue

            linear_x  = message.linear_velocity_x
            angular_z = message.angular_velocity

            # ── Zaman delta ────────────────────────
            now_msg = self.get_clock().now()
            now_sec = now_msg.nanoseconds / 1e9

            if self.last_time is not None:
                dt = now_sec - self.last_time
                # Aşırı büyük dt'leri atla (ilk mesaj veya gecikme)
                if 0.0 < dt < 1.0:
                    self.x   += linear_x * math.cos(self.yaw) * dt
                    self.y   += linear_x * math.sin(self.yaw) * dt
                    self.yaw += angular_z * dt
                    # Yaw'ı [-pi, pi] arasında tut
                    self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

            self.last_time = now_sec

            # ── Quaternion ─────────────────────────
            q = quaternion_from_euler(0.0, 0.0, self.yaw)

            # ── Odometry mesajı ────────────────────
            odom_msg = Odometry()
            odom_msg.header.stamp    = now_msg.to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id  = "base_link"

            # Pose
            odom_msg.pose.pose.position.x  = self.x
            odom_msg.pose.pose.position.y  = self.y
            odom_msg.pose.pose.position.z  = 0.0
            odom_msg.pose.pose.orientation = Quaternion(
                x=q[0], y=q[1], z=q[2], w=q[3]
            )
            # Pose covariance — dead reckoning zamanla birikim gösterir
            pose_cov       = [0.0] * 36
            pose_cov[0]    = 0.1    # x
            pose_cov[7]    = 0.1    # y
            pose_cov[14]   = 1e6    # z (bilmiyoruz)
            pose_cov[21]   = 1e6    # roll
            pose_cov[28]   = 1e6    # pitch
            pose_cov[35]   = 0.05   # yaw
            odom_msg.pose.covariance = pose_cov

            # Twist
            odom_msg.twist.twist = Twist()
            odom_msg.twist.twist.linear.x  = linear_x
            odom_msg.twist.twist.angular.z = angular_z

            twist_cov      = [0.0] * 36
            twist_cov[0]   = 0.01    # vx
            twist_cov[7]   = 0.01    # vy
            twist_cov[14]  = 1e6     # vz
            twist_cov[21]  = 1e6     # wx
            twist_cov[28]  = 1e6     # wy
            twist_cov[35]  = 0.01    # wz
            odom_msg.twist.covariance = twist_cov

            # Yayınla
            self.pub.publish(odom_msg)

            # CSV log
            t = odom_msg.header.stamp
            with open(self.log_file, mode="a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    t.sec, t.nanosec,
                    linear_x, angular_z,
                    round(self.x, 4), round(self.y, 4), round(self.yaw, 4)
                ])

            self.get_logger().info(
                f"vx={linear_x:.3f} m/s  wz={angular_z:.3f} r/s  "
                f"x={self.x:.2f}  y={self.y:.2f}  yaw={math.degrees(self.yaw):.1f}°"
            )


def main():
    parser = argparse.ArgumentParser(description="Amiga Odometry Publisher")
    parser.add_argument("--service-config", type=Path, required=True,
                        help="Motor service config JSON")
    parser.add_argument("--log-file", type=Path, default="velocity_log.csv",
                        help="CSV log dosyası")
    args = parser.parse_args()

    rclpy.init()
    config = proto_from_json_file(args.service_config, EventServiceConfig())
    node   = OdomPublisher(config, args.log_file)

    try:
        asyncio.run(node.run())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()