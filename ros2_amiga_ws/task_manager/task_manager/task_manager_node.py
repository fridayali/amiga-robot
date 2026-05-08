"""
Task Manager Node — manages mission queues and sends navigation goals to Nav2.

Topics subscribed:
  /task_manager/add_waypoint  (geometry_msgs/PoseStamped)  — enqueue a waypoint
  /task_manager/cancel        (std_msgs/Empty)             — cancel current mission
  /mission_segments           (std_msgs/String)            — JSON mission from websocket_bridge

Topics published:
  /task_manager/status        (std_msgs/String)            — current task status
  /task_manager/current_goal  (geometry_msgs/PoseStamped)  — active navigation goal
  /plow_command               (std_msgs/String)            — "down" or "up"

Services:
  /task_manager/clear_queue   (std_srvs/Trigger)           — clear all queued tasks
  /task_manager/pause         (std_srvs/SetBool)           — pause/resume execution
"""

import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from robot_localization.srv import FromLL

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

        # Service client: GPS lat/lon → map frame Point
        self._from_ll_client = self.create_client(
            FromLL, '/fromLL', callback_group=self._cb_group)

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

        # Services
        self.create_service(Trigger, '/task_manager/clear_queue', self._srv_clear_queue)
        self.create_service(SetBool, '/task_manager/pause',       self._srv_pause)

        # Queue items: PoseStamped (nav goal) or dict {'action': 'plow_down'/'plow_up'}
        self._queue: deque = deque()
        self._state = TaskState.IDLE
        self._current_goal_handle = None
        self._mission_meta: dict = {}

        self.create_timer(0.5, self._dispatch_timer_cb, callback_group=self._cb_group)

        self.get_logger().info('TaskManagerNode started.')

    # ------------------------------------------------------------------ #
    #  Mission Segments (from websocket_bridge → /mission_segments)       #
    # ------------------------------------------------------------------ #

    def _cb_mission_segments(self, msg: String):
        # Load mission in a separate thread to avoid blocking the executor
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
                pose = self._gps_to_pose(seg['latitude'], seg['longitude'])
                if pose:
                    new_queue.append(pose)
                else:
                    self.get_logger().error(
                        f"fromLL failed for segment {seg['order_index']}, skipping.")
            elif action in ('plow_down', 'plow_up'):
                new_queue.append({'action': action})

        self._queue = new_queue
        self._state = TaskState.IDLE
        self.get_logger().info(
            f"Mission {self._mission_meta.get('mission_id')} loaded: "
            f"{len(new_queue)} steps queued.")
        self._publish_status()

    def _gps_to_pose(self, lat: float, lon: float):
        """Call /fromLL to convert GPS coordinates to map-frame PoseStamped."""
        if not self._from_ll_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/fromLL service not available.')
            return None

        req = FromLL.Request()
        req.ll_point.latitude  = float(lat)
        req.ll_point.longitude = float(lon)
        req.ll_point.altitude  = 0.0

        event = threading.Event()
        result_holder: list = [None]

        def _done(future):
            result_holder[0] = future.result()
            event.set()

        self._from_ll_client.call_async(req).add_done_callback(_done)

        if not event.wait(timeout=5.0):
            self.get_logger().error(f'fromLL timed out for ({lat}, {lon}).')
            return None

        pt = result_holder[0].map_point
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = pt.x
        pose.pose.position.y = pt.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    # ------------------------------------------------------------------ #
    #  Callbacks                                                           #
    # ------------------------------------------------------------------ #

    def _cb_add_waypoint(self, msg: PoseStamped):
        self._queue.append(msg)
        self.get_logger().info(
            f'Waypoint enqueued. Queue length: {len(self._queue)}')
        self._publish_status()

    def _cb_cancel(self, _: Empty):
        if self._current_goal_handle is not None:
            self.get_logger().info('Canceling current navigation goal.')
            self._state = TaskState.CANCELING
            self._current_goal_handle.cancel_goal_async()
        self._queue.clear()
        self._publish_status()

    def _srv_clear_queue(self, _req, response):
        self._queue.clear()
        self.get_logger().info('Task queue cleared.')
        response.success = True
        response.message = 'Queue cleared.'
        self._publish_status()
        return response

    def _srv_pause(self, req, response):
        if req.data:
            self._state = TaskState.PAUSED
            self.get_logger().info('Task execution paused.')
        else:
            if self._state == TaskState.PAUSED:
                self._state = TaskState.IDLE
                self.get_logger().info('Task execution resumed.')
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
                self.get_logger().warn('NavigateToPose action server not available yet.')
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
            f'Nav2 goal: x={pose.pose.position.x:.3f} y={pose.pose.position.y:.3f}')

        send_future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

    def _send_plow_command(self, action: str):
        cmd = 'down' if action == 'plow_down' else 'up'
        msg = String()
        msg.data = cmd
        self._plow_pub.publish(msg)
        self.get_logger().info(f'Plow command sent: {cmd}')
        # Fire-and-forget; IDLE state is preserved so next item dispatches immediately

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2.')
            self._state = TaskState.IDLE
            self._publish_status()
            return
        self._current_goal_handle = handle
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        self._current_goal_handle = None

        if self._state == TaskState.CANCELING:
            self.get_logger().info('Goal canceled.')
        else:
            self.get_logger().info(f'Goal reached. Status: {future.result().status}')

        self._state = TaskState.IDLE
        self._publish_status()

    def _feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().debug(f'Distance remaining: {dist:.2f} m')

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
