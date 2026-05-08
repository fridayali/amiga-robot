"""
Task Manager Node — manages mission queues and sends navigation goals to Nav2.

Topics subscribed:
  /task_manager/add_waypoint  (geometry_msgs/PoseStamped)  — enqueue a waypoint
  /task_manager/cancel        (std_msgs/Empty)             — cancel current mission
  /mission_segments           (std_msgs/String)            — JSON mission from websocket_bridge
  /gps/fix                    (sensor_msgs/NavSatFix)      — current GPS (RTK referansı için)
  /rtk/odom                   (nav_msgs/Odometry)          — RTK map frame pozisyonu

Topics published:
  /task_manager/status        (std_msgs/String)            — current task status
  /task_manager/current_goal  (geometry_msgs/PoseStamped)  — active navigation goal
  /plow_command               (std_msgs/String)            — "down" or "up"

Services:
  /task_manager/clear_queue   (std_srvs/Trigger)           — clear all queued tasks
  /task_manager/pause         (std_srvs/SetBool)           — pause/resume execution
"""

import json
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from nav2_msgs.action import NavigateToPose

from collections import deque
from enum import Enum, auto


class TaskState(Enum):
    IDLE      = auto()
    RUNNING   = auto()
    PAUSED    = auto()
    CANCELING = auto()


class TaskManagerNode(Node):

    def __init__(self):
        super().__init__('task_manager')

        self._cb_group = ReentrantCallbackGroup()

        # Action client for Nav2 NavigateToPose
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cb_group)

        # Publishers
        self._status_pub = self.create_publisher(String, '/task_manager/status', 10)
        self._goal_pub   = self.create_publisher(PoseStamped, '/task_manager/current_goal', 10)
        self._plow_pub   = self.create_publisher(String, '/plow_command', 10)

        # Subscribers
        self.create_subscription(PoseStamped, '/task_manager/add_waypoint',
                                 self._cb_add_waypoint, 10,
                                 callback_group=self._cb_group)
        self.create_subscription(Empty, '/task_manager/cancel',
                                 self._cb_cancel, 10,
                                 callback_group=self._cb_group)
        self.create_subscription(String, '/mission_segments',
                                 self._cb_mission_segments, 10,
                                 callback_group=self._cb_group)

        # RTK referans noktası için GPS ve odom
        self.create_subscription(NavSatFix, '/gps/fix',
                                 self._cb_gps, 10,
                                 callback_group=self._cb_group)
        self.create_subscription(Odometry, '/rtk/odom',
                                 self._cb_rtk_odom, 10,
                                 callback_group=self._cb_group)

        # Services
        self.create_service(Trigger, '/task_manager/clear_queue', self._srv_clear_queue)
        self.create_service(SetBool, '/task_manager/pause',       self._srv_pause)

        # RTK referans: (lat0, lon0) ↔ (x0, y0) in map frame
        self._ref_gps: tuple | None = None   # (lat, lon)
        self._ref_map: tuple | None = None   # (x, y)
        self._latest_gps: tuple | None = None
        self._latest_map: tuple | None = None

        # Queue items: PoseStamped (nav goal) or dict {'action': 'plow_down'/'plow_up'}
        self._queue: deque = deque()
        self._state = TaskState.IDLE
        self._current_goal_handle = None
        self._mission_meta: dict = {}

        self.create_timer(0.5, self._dispatch_timer_cb, callback_group=self._cb_group)

        self.get_logger().info('TaskManagerNode started.')

    # ------------------------------------------------------------------ #
    #  RTK Referans                                                        #
    # ------------------------------------------------------------------ #

    def _cb_gps(self, msg: NavSatFix):
        if msg.status.status < 0:
            return
        self._latest_gps = (msg.latitude, msg.longitude)

    def _cb_rtk_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._latest_map = (p.x, p.y)

    def _ensure_ref(self) -> bool:
        """İlk çağrıda referans noktasını sabitle."""
        if self._ref_gps is not None:
            return True
        if self._latest_gps is None or self._latest_map is None:
            return False
        self._ref_gps = self._latest_gps
        self._ref_map = self._latest_map
        self.get_logger().info(
            f'RTK referans noktası alındı: '
            f'lat={self._ref_gps[0]:.7f} lon={self._ref_gps[1]:.7f} '
            f'→ map x={self._ref_map[0]:.3f} y={self._ref_map[1]:.3f}')
        return True

    def _latlon_to_map(self, lat: float, lon: float):
        """Lat/lon → map frame PoseStamped (RTK referansına göre ENU dönüşümü)."""
        if not self._ensure_ref():
            self.get_logger().error('RTK referans noktası henüz yok, GPS/odom bekleniyor.')
            return None

        lat0, lon0 = self._ref_gps
        x0,   y0   = self._ref_map
        R = 6378137.0  # WGS-84 yarıçapı

        dx = R * math.radians(lon - lon0) * math.cos(math.radians(lat0))  # doğu
        dy = R * math.radians(lat - lat0)                                  # kuzey

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = x0 + dx
        pose.pose.position.y = y0 + dy
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    # ------------------------------------------------------------------ #
    #  Mission Segments (from websocket_bridge → /mission_segments)       #
    # ------------------------------------------------------------------ #

    def _cb_mission_segments(self, msg: String):
        threading.Thread(target=self._load_mission, args=(msg.data,), daemon=True).start()

    def _load_mission(self, json_str: str):
        try:
            data = json.loads(json_str)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid mission JSON: {e}')
            return

        self._mission_meta = {
            'mission_id': data.get('mission_id'),
            'field_id':   data.get('field_id'),
            'zone_id':    data.get('zone_id'),
        }

        segments = sorted(data.get('segments_json', []),
                          key=lambda s: s['order_index'])

        new_queue: deque = deque()
        for seg in segments:
            action = seg.get('action')
            if action == 'move':
                pose = self._latlon_to_map(seg['latitude'], seg['longitude'])
                if pose:
                    new_queue.append(pose)
                else:
                    self.get_logger().error(
                        f"Dönüşüm başarısız: segment {seg['order_index']} atlandı.")
            elif action in ('plow_down', 'plow_up'):
                new_queue.append({'action': action})

        self._queue = new_queue
        self._state = TaskState.IDLE
        self.get_logger().info(
            f"Mission {self._mission_meta.get('mission_id')} yüklendi: "
            f"{len(new_queue)} adım.")
        self._publish_status()

    # ------------------------------------------------------------------ #
    #  Callbacks                                                           #
    # ------------------------------------------------------------------ #

    def _cb_add_waypoint(self, msg: PoseStamped):
        self._queue.append(msg)
        self.get_logger().info(f'Waypoint eklendi. Kuyruk: {len(self._queue)}')
        self._publish_status()

    def _cb_cancel(self, _: Empty):
        if self._current_goal_handle is not None:
            self.get_logger().info('Mevcut navigasyon hedefi iptal ediliyor.')
            self._state = TaskState.CANCELING
            self._current_goal_handle.cancel_goal_async()
        self._queue.clear()
        self._publish_status()

    def _srv_clear_queue(self, _req, response):
        self._queue.clear()
        self.get_logger().info('Kuyruk temizlendi.')
        response.success = True
        response.message = 'Queue cleared.'
        self._publish_status()
        return response

    def _srv_pause(self, req, response):
        if req.data:
            self._state = TaskState.PAUSED
            self.get_logger().info('Görev duraklatıldı.')
        else:
            if self._state == TaskState.PAUSED:
                self._state = TaskState.IDLE
                self.get_logger().info('Görev devam ediyor.')
        response.success = True
        response.message = f'State: {self._state.name}'
        self._publish_status()
        return response

    # ------------------------------------------------------------------ #
    #  Dispatch logic                                                      #
    # ------------------------------------------------------------------ #

    def _dispatch_timer_cb(self):
        if self._state != TaskState.IDLE:
            return
        if not self._queue:
            return

        item = self._queue.popleft()

        if isinstance(item, PoseStamped):
            if not self._nav_client.wait_for_server(timeout_sec=0.0):
                self.get_logger().warn('NavigateToPose action server hazır değil.')
                self._queue.appendleft(item)
                return
            self._send_nav_goal(item)
        elif isinstance(item, dict):
            self._send_plow_command(item['action'])

    def _send_nav_goal(self, pose: PoseStamped):
        self._state = TaskState.RUNNING
        self._goal_pub.publish(pose)
        self._publish_status()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            f'Nav2 hedef: x={pose.pose.position.x:.3f} y={pose.pose.position.y:.3f}')

        send_future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

    def _send_plow_command(self, action: str):
        cmd = 'down' if action == 'plow_down' else 'up'
        msg = String()
        msg.data = cmd
        self._plow_pub.publish(msg)
        self.get_logger().info(f'Plow komutu: {cmd}')

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 hedefi reddetti.')
            self._state = TaskState.IDLE
            self._publish_status()
            return
        self._current_goal_handle = handle
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        self._current_goal_handle = None
        status = future.result().status
        if self._state == TaskState.CANCELING or status == 5:
            self.get_logger().info('Hedef iptal edildi.')
        elif status == 4:
            self.get_logger().info('Hedefe ulaşıldı.')
        else:
            self.get_logger().error(f'Nav2 hedef ABORT etti! Status: {status} — TF/map/costmap kontrol et.')
        self._state = TaskState.IDLE
        self._publish_status()

    def _feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().debug(f'Kalan mesafe: {dist:.2f} m')

    # ------------------------------------------------------------------ #
    #  Helpers                                                             #
    # ------------------------------------------------------------------ #

    def _publish_status(self):
        msg = String()
        msg.data = (
            f'state={self._state.name} '
            f'queue_size={len(self._queue)} '
            f'mission_id={self._mission_meta.get("mission_id", "none")}'
        )
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
