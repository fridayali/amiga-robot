#!/usr/bin/env python3
"""
Task Manager Node — farm-ng track follower ile GPS waypoint tabanlı görev yöneticisi.

Mission JSON formatı (/mission_segments):
  {
    "mission_id": "...",
    "segments_json": [
      {"order_index": 0, "action": "move", "gpsWaypoints": [{"latitude":..., "longitude":..., "altitude":...}]},
      {"order_index": 1, "action": "plow_down", "duration": 12},
      {"order_index": 2, "action": "move", "latitude": ..., "longitude": ...},
      {"order_index": 3, "action": "plow_up",   "duration": 14}
    ]
  }

  track_path verilirse o dosyayı okur ve içeriğini track_executor'a iletir.
  gpsWaypoints verilirse doğrudan track_executor'a JSON olarak gönderir.

Topics subscribed:
  /mission_segments      (std_msgs/String)  — JSON mission
  /task_manager/cancel   (std_msgs/Empty)   — anlık görevi iptal et

Topics published:
  /task_manager/status   (std_msgs/String)  — durum

Services:
  /task_manager/clear_queue (std_srvs/Trigger)  — kuyruğu temizle + iptal et
  /task_manager/pause       (std_srvs/SetBool)  — duraklat / devam et

Parametreler:
  track_follower_config  — track follower servis config JSON yolu
  track_executor_python  — farm-ng venv python3 tam yolu
  track_executor_script  — track_executor.py tam yolu
"""

import asyncio
import json
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from std_msgs.msg import Empty as RosEmpty
from std_srvs.srv import Trigger, SetBool

# Varsayılan plow süreleri (segment'te "duration" yoksa bunlar kullanılır)
_PLOW_DOWN_SEC = 12.0
_PLOW_UP_SEC   = 14.0


class TaskManagerNode(Node):

    def __init__(self):
        super().__init__('task_manager')

        self._cb_group = ReentrantCallbackGroup()

        self.declare_parameter(
            'track_follower_config',
            '/ros2_amiga_ws/src/ros2_bridge/config/track_follower.json')
        self.declare_parameter(
            'track_executor_python',
            '/mnt/managed_home/farm-ng-user-ertugrulkalkan/farm-ng-amiga/venv/bin/python3')
        self.declare_parameter(
            'track_executor_script',
            '/ros2_amiga_ws/src/ros2_bridge/src/track_executor.py')

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

        # Asyncio loop
        self._loop:          asyncio.AbstractEventLoop = asyncio.new_event_loop()
        self._mission_queue: asyncio.Queue | None      = None
        self._cancel_event:  asyncio.Event | None      = None
        self._pause_event:   asyncio.Event | None      = None
        self._move_proc:     asyncio.subprocess.Process | None = None

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

        cfg  = self.get_parameter('track_follower_config').value
        py   = self.get_parameter('track_executor_python').value
        scr  = self.get_parameter('track_executor_script').value

        self.get_logger().info(
            f'track_follower_config : {cfg}\n'
            f'track_executor_python : {py}\n'
            f'track_executor_script : {scr}\n'
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

            if self._cancel_event.is_set():
                self.get_logger().info('Mission iptal edildi.')
                return

            await self._pause_event.wait()

            action = seg.get('action', '').lower()
            self.get_logger().info(
                f'┌─ Adım {i+1}/{total}: [{action.upper()}] '
                f'──────────────────────────')

            if action == 'move':
                await self._step_move(seg)

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
    #  Adım: Move  (track_executor.py aracılığıyla)
    # ──────────────────────────────────────────────────

    async def _step_move(self, seg: dict):
        track_path = seg.get('track_path')

        if track_path:
            self.get_logger().info(f'│  Track dosyası: {track_path}')
            track_json = Path(track_path).read_text()
            track_data = json.loads(track_json)
        else:
            gps_wps = seg.get('gpsWaypoints') or [
                {'latitude': seg['latitude'], 'longitude': seg['longitude'],
                 'altitude': seg.get('altitude', 0.0)}
            ]
            self.get_logger().info(
                f'│  GPS waypoints ({len(gps_wps)} nokta):\n' +
                '\n'.join(
                    f'│    [{i}] lat={w["latitude"]:.7f}  lon={w["longitude"]:.7f}'
                    for i, w in enumerate(gps_wps)))
            track_data = {'gpsWaypoints': gps_wps}

        cfg = self.get_parameter('track_follower_config').value
        py  = self.get_parameter('track_executor_python').value
        scr = self.get_parameter('track_executor_script').value

        stdin_bytes = json.dumps(track_data).encode()

        self.get_logger().info('│  → track_executor başlatılıyor...')

        proc = await asyncio.create_subprocess_exec(
            py, scr, cfg,
            stdin=asyncio.subprocess.PIPE,
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.PIPE,
        )
        self._move_proc = proc

        try:
            _, stderr_bytes = await proc.communicate(input=stdin_bytes)
        except asyncio.CancelledError:
            proc.terminate()
            raise
        finally:
            self._move_proc = None

        for line in (stderr_bytes or b'').decode().splitlines():
            self.get_logger().info('│  [executor] ' + line)

        rc = proc.returncode
        if rc == 0:
            self.get_logger().info('│  ✓ Hedefe ulaşıldı.')
        elif rc == 2:
            self.get_logger().info('│  Track iptal edildi.')
        else:
            raise RuntimeError(f'track_executor hata kodu: {rc}')

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
                + (f' ({len(s.get("gpsWaypoints", []))} wp)'
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
        # Çalışan move process'i sonlandır
        if self._move_proc and self._move_proc.returncode is None:
            self._move_proc.terminate()
        self.get_logger().info('İPTAL sinyali alındı.')
        self._publish_status('CANCELING', 0)

    def _srv_clear(self, _req, response):
        if self._cancel_event:
            self._cancel_event.set()
        if self._move_proc and self._move_proc.returncode is None:
            self._move_proc.terminate()
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
