#!/usr/bin/env python3
"""
Task Manager Node — farm-ng track follower ile GPS waypoint tabanlı görev yöneticisi.

Mission JSON formatı (/mission_segments):
  {
    "mission_id": "...",
    "segments_json": [
      {"order_index": 0, "action": "move",      "latitude": ..., "longitude": ..., "track_path": "(opsiyonel)"},
      {"order_index": 1, "action": "plow_down", "duration": 12},
      {"order_index": 2, "action": "move",      "latitude": ..., "longitude": ...},
      {"order_index": 3, "action": "plow_up",   "duration": 14}
    ]
  }

  track_path verilirse o dosyayı yükler; verilmezse GPS → ENU dönüşümü ile track oluşturur.

Topics subscribed:
  /mission_segments      (std_msgs/String)  — JSON mission
  /task_manager/cancel   (std_msgs/Empty)   — anlık görevi iptal et

Topics published:
  /task_manager/status   (std_msgs/String)  — durum

Services:
  /task_manager/clear_queue (std_srvs/Trigger)  — kuyruğu temizle + iptal et
  /task_manager/pause       (std_srvs/SetBool)  — duraklat / devam et
"""

import asyncio
import json
import math
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from std_msgs.msg import Empty as RosEmpty
from std_srvs.srv import Trigger, SetBool

from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file
from farm_ng.track.track_pb2 import Track, TrackFollowRequest, TRACK_COMPLETE
from google.protobuf.empty_pb2 import Empty as ProtoEmpty

# navsat_transform datum ile aynı — ENU referans noktası
_DATUM_LAT = 39.79609718
_DATUM_LON = 32.53153558

# Varsayılan plow süreleri (segment'te "duration" yoksa bunlar kullanılır)
_PLOW_DOWN_SEC = 12.0
_PLOW_UP_SEC   = 14.0


def _gps_to_enu(lat: float, lon: float) -> tuple[float, float]:
    R = 6378137.0
    dx = R * math.radians(lon - _DATUM_LON) * math.cos(math.radians(_DATUM_LAT))
    dy = R * math.radians(lat - _DATUM_LAT)
    return dx, dy


def _build_track_from_enu(x: float, y: float) -> Track:
    """ENU koordinatından tek noktalı Track oluşturur."""
    track = Track()
    wp = track.waypoints.add()
    wp.x       = x
    wp.y       = y
    wp.heading = 0.0
    return track


def _load_track_from_file(path: str) -> Track:
    return proto_from_json_file(Path(path), Track())


class TaskManagerNode(Node):

    def __init__(self):
        super().__init__('task_manager')

        self._cb_group = ReentrantCallbackGroup()

        self.declare_parameter(
            'track_follower_config',
            '/home/cuma_karaaslan/farm-ng-amiga/py/examples/track_follower/service_config.json')

        # Publishers
        self._status_pub = self.create_publisher(String, '/task_manager/status', 10)

        # Subscribers
        self.create_subscription(String, '/mission_segments',
                                 self._cb_mission_segments, 10,
                                 callback_group=self._cb_group)
        self.create_subscription(RosEmpty, '/task_manager/cancel',
                                 self._cb_cancel, 10,
                                 callback_group=self._cb_group)

        # Services
        self.create_service(Trigger, '/task_manager/clear_queue', self._srv_clear)
        self.create_service(SetBool, '/task_manager/pause',       self._srv_pause)

        # Asyncio loop (farm-ng tüm operasyonları burada çalışır)
        self._loop:          asyncio.AbstractEventLoop = asyncio.new_event_loop()
        self._mission_queue: asyncio.Queue | None      = None
        self._cancel_event:  asyncio.Event | None      = None
        self._pause_event:   asyncio.Event | None      = None
        self._tf_config:     EventServiceConfig | None = None

        threading.Thread(target=self._run_async_loop, daemon=True).start()

        self.get_logger().info('TaskManagerNode başlatıldı.')

    # ──────────────────────────────────────────────────
    #  Asyncio altyapısı
    # ──────────────────────────────────────────────────

    def _run_async_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._async_main())

    async def _async_main(self):
        self._mission_queue = asyncio.Queue()
        self._cancel_event  = asyncio.Event()
        self._pause_event   = asyncio.Event()
        self._pause_event.set()  # başlangıçta çalışıyor

        cfg_path = self.get_parameter('track_follower_config').value
        self._tf_config = proto_from_json_file(Path(cfg_path), EventServiceConfig())

        self.get_logger().info(
            f'Track follower config: {cfg_path}\n'
            f'Datum: lat={_DATUM_LAT}  lon={_DATUM_LON}\n'
            f'Mission bekleniyor...')

        await self._mission_loop()

    # ──────────────────────────────────────────────────
    #  Mission döngüsü
    # ──────────────────────────────────────────────────

    async def _mission_loop(self):
        while True:
            segments = await self._mission_queue.get()
            self._cancel_event.clear()

            self.get_logger().info(
                '╔══════════════════════════════════╗\n'
                f'║  YENİ MİSSİON — {len(segments):2d} adım          ║\n'
                '╚══════════════════════════════════╝')
            self._publish_status('RUNNING', len(segments))

            try:
                await self._execute_mission(segments)
            except Exception as e:
                self.get_logger().error(f'Mission istisnası: {e}')

            self.get_logger().info('Mission döngüsü bitti. Yeni mission bekleniyor.')
            self._publish_status('IDLE', 0)

    async def _execute_mission(self, segments: list):
        total = len(segments)

        for i, seg in enumerate(segments):

            # İptal kontrolü
            if self._cancel_event.is_set():
                self.get_logger().info('Mission iptal edildi.')
                return

            # Pause bekle
            await self._pause_event.wait()

            action = seg.get('action', '').lower()
            self.get_logger().info(
                f'┌─ Adım {i+1}/{total}: [{action.upper()}] '
                f'──────────────────────────')

            if action == 'move':
                await self._step_move(seg, i + 1, total)

            elif action == 'plow_down':
                dur = float(seg.get('duration', _PLOW_DOWN_SEC))
                await self._step_plow('forward', dur, 'DOWN')

            elif action == 'plow_up':
                dur = float(seg.get('duration', _PLOW_UP_SEC))
                await self._step_plow('reverse', dur, 'UP')

            else:
                self.get_logger().warn(f'└─ Bilinmeyen action "{action}", atlandı.')
                continue

            self.get_logger().info(f'└─ Adım {i+1}/{total} tamamlandı.')

        self.get_logger().info(
            '╔══════════════════════════════════╗\n'
            '║  TÜM ADIMLAR TAMAMLANDI ✓        ║\n'
            '╚══════════════════════════════════╝')

    # ──────────────────────────────────────────────────
    #  Adım: Move
    # ──────────────────────────────────────────────────

    async def _step_move(self, seg: dict, step: int, total: int):
        track_path = seg.get('track_path')

        if track_path:
            self.get_logger().info(f'│  Track dosyası: {track_path}')
            track = _load_track_from_file(track_path)
        else:
            lat = seg['latitude']
            lon = seg['longitude']
            dx, dy = _gps_to_enu(lat, lon)
            self.get_logger().info(
                f'│  GPS : lat={lat:.7f}  lon={lon:.7f}\n'
                f'│  ENU : x={dx:.3f} m  y={dy:.3f} m')
            track = _build_track_from_enu(dx, dy)

        client = EventClient(self._tf_config)

        self.get_logger().info('│  → /set_track gönderiliyor...')
        await client.request_reply('/set_track', TrackFollowRequest(track=track))

        self.get_logger().info('│  → /start gönderiliyor...')
        await client.request_reply('/start', ProtoEmpty())

        self.get_logger().info('│  Track execute ediliyor, tamamlanması bekleniyor...')
        await self._wait_track_complete()

        self.get_logger().info('│  ✓ Hedefe ulaşıldı.')

    async def _wait_track_complete(self):
        client = EventClient(self._tf_config)
        async for _ev, msg in client.subscribe(
                self._tf_config.subscriptions[0], decode=True):

            if self._cancel_event.is_set():
                self.get_logger().info('│  Track izleme iptal edildi.')
                return

            try:
                if msg.status.track_status == TRACK_COMPLETE:
                    return
                dist = getattr(msg, 'distance_remaining', None)
                if dist is not None:
                    self.get_logger().debug(f'│  Kalan mesafe: {dist:.2f} m')
            except Exception:
                pass

    # ──────────────────────────────────────────────────
    #  Adım: Plow
    # ──────────────────────────────────────────────────

    async def _step_plow(self, direction: str, duration: float, label: str):
        from tool_control.main import send_hbridge_command

        self.get_logger().info(
            f'│  Tool {label} başlıyor — yön={direction}  süre={duration:.1f}s')

        start = time.time()
        while time.time() - start < duration:
            if self._cancel_event.is_set():
                self.get_logger().info('│  Plow komutu iptal edildi.')
                break
            await send_hbridge_command(direction)
            await asyncio.sleep(0.1)

        await send_hbridge_command('stop')
        elapsed = time.time() - start
        self.get_logger().info(
            f'│  ✓ Tool {label} tamamlandı ({elapsed:.1f}s çalıştı).')

    # ──────────────────────────────────────────────────
    #  ROS2 callbacks & services
    # ──────────────────────────────────────────────────

    def _cb_mission_segments(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON parse hatası: {e}')
            return

        segments = sorted(
            data.get('segments_json', []),
            key=lambda s: s['order_index'])

        if not segments:
            self.get_logger().warn('Boş mission alındı.')
            return

        mid = data.get('mission_id', 'N/A')
        self.get_logger().info(
            f'Mission alındı → id={mid}  adım={len(segments)}\n'
            + '\n'.join(
                f'  [{s["order_index"]}] {s["action"]}'
                + (f' lat={s["latitude"]:.6f} lon={s["longitude"]:.6f}'
                   if s["action"] == "move" else
                   f' dur={s.get("duration", "default")}s')
                for s in segments))

        if self._mission_queue is None:
            self.get_logger().error('Asyncio loop henüz hazır değil!')
            return

        asyncio.run_coroutine_threadsafe(
            self._mission_queue.put(segments), self._loop)

    def _cb_cancel(self, _: RosEmpty):
        if self._cancel_event:
            self._cancel_event.set()
        self.get_logger().info('İPTAL sinyali alındı.')
        self._publish_status('CANCELING', 0)

    def _srv_clear(self, _req, response):
        if self._cancel_event:
            self._cancel_event.set()
        if self._mission_queue:
            while not self._mission_queue.empty():
                try:
                    self._mission_queue.get_nowait()
                except Exception:
                    break
        self.get_logger().info('Kuyruk temizlendi, görev iptal edildi.')
        response.success = True
        response.message = 'Kuyruk temizlendi.'
        self._publish_status('IDLE', 0)
        return response

    def _srv_pause(self, req, response):
        if req.data:
            self._pause_event.clear()
            self.get_logger().info('Görev duraklatıldı.')
            self._publish_status('PAUSED', 0)
        else:
            self._pause_event.set()
            self.get_logger().info('Görev devam ediyor.')
            self._publish_status('RUNNING', 0)
        response.success = True
        response.message = 'Duraklatıldı.' if req.data else 'Devam ediyor.'
        return response

    def _publish_status(self, state: str, queue_size: int):
        msg = String()
        msg.data = f'state={state} queue_size={queue_size}'
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
