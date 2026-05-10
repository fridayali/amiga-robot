#!/usr/bin/env python3
"""
Track Executor — farm-ng venv ile çalıştırılır.

Stdin'den track JSON okur, track follower'a gönderir, tamamlanmasını bekler.
Kullanım:
  /home/farm-ng-user-ertugrulkalkan/farm-ng-amiga/venv/bin/python3 track_executor.py <config.json>
  (stdin'e track JSON gönderilir)

Çıkış kodu:
  0 — TRACK_COMPLETE
  1 — Hata / bağlantı sorunu
  2 — İptal sinyali (SIGINT)
"""
import asyncio
import json
import signal
import sys
from pathlib import Path

from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file
from farm_ng.track.track_pb2 import Track, TrackFollowerState, TrackFollowRequest, TRACK_COMPLETE
from google.protobuf.empty_pb2 import Empty as ProtoEmpty
from google.protobuf import json_format

_TERMINAL = {6: 'TRACK_FAILED', 7: 'TRACK_ABORTED', 8: 'TRACK_CANCELLED'}
_FAILURE_MODES = {10: 'CANBUS_TIMEOUT', 11: 'AUTO_MODE_DISABLED',
                  12: 'CANBUS_SEND_ERROR', 20: 'FILTER_TIMEOUT', 21: 'FILTER_DIVERGED'}


def _build_track(track_data: dict) -> Track:
    return json_format.ParseDict(track_data, Track())


async def _run(config_path: str):
    raw = sys.stdin.read().strip()
    if not raw:
        print('[track_executor] HATA: stdin boş', file=sys.stderr)
        return 1

    try:
        track_data = json.loads(raw)
    except json.JSONDecodeError as e:
        print(f'[track_executor] JSON parse hatası: {e}', file=sys.stderr)
        return 1

    config = proto_from_json_file(Path(config_path), EventServiceConfig())
    track  = _build_track(track_data)

    if len(track.gps_waypoints) == 0:
        print('[track_executor] HATA: GPS waypoint yok', file=sys.stderr)
        return 1

    print(f'[track_executor] {len(track.gps_waypoints)} waypoint yükleniyor...',
          file=sys.stderr)

    client = EventClient(config)

    try:
        await client.request_reply('/set_track', TrackFollowRequest(track=track))
        print('[track_executor] /set_track gönderildi', file=sys.stderr)

        await client.request_reply('/start', ProtoEmpty())
        print('[track_executor] /start gönderildi, tamamlanma bekleniyor...',
              file=sys.stderr)

        _last_controllable = True

        async for _ev, msg in client.subscribe(config.subscriptions[0], decode=True):
            status = msg.status.track_status

            if status == TRACK_COMPLETE:
                print('[track_executor] TRACK_COMPLETE ✓', file=sys.stderr)
                return 0

            if status in _TERMINAL:
                print(f'[track_executor] Terminal durum: {_TERMINAL[status]}', file=sys.stderr)
                rs = msg.status.robot_status
                if not rs.controllable:
                    for fm in rs.failure_modes:
                        print(f'[track_executor]   {_FAILURE_MODES.get(fm, f"UNKNOWN_{fm}")}',
                              file=sys.stderr)
                return 1

            # Robot kontrolsüz olunca / tekrar kontrolsüz olunca logla
            rs = msg.status.robot_status
            if not rs.controllable and _last_controllable:
                modes = ', '.join(_FAILURE_MODES.get(fm, f'UNKNOWN_{fm}')
                                  for fm in rs.failure_modes)
                print(f'[track_executor] UYARI: Robot kontrolsüz — {modes}', file=sys.stderr)
            elif rs.controllable and not _last_controllable:
                print('[track_executor] Robot tekrar kontrol edilebilir.', file=sys.stderr)
            _last_controllable = rs.controllable

            try:
                rem = msg.progress.distance_remaining
                print(f'[track_executor] kalan: {rem:.2f} m', file=sys.stderr)
            except Exception:
                pass

    except asyncio.CancelledError:
        print('[track_executor] İptal edildi', file=sys.stderr)
        return 2
    except Exception as e:
        print(f'[track_executor] HATA: {e}', file=sys.stderr)
        return 1

    return 0


def main():
    if len(sys.argv) < 2:
        print('Kullanım: track_executor.py <track_follower_config.json>', file=sys.stderr)
        sys.exit(1)

    config_path = sys.argv[1]

    loop = asyncio.new_event_loop()

    def _sigint(_sig, _frame):
        print('[track_executor] SIGINT alındı', file=sys.stderr)
        loop.stop()

    signal.signal(signal.SIGINT, _sigint)

    try:
        rc = loop.run_until_complete(_run(config_path))
    except Exception as e:
        print(f'[track_executor] Fatal: {e}', file=sys.stderr)
        rc = 1
    finally:
        loop.close()

    sys.exit(rc)


if __name__ == '__main__':
    main()
