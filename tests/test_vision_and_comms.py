"""vision_engine + comms_gateway — sentetik kamera/YOLO ve hakem kuyruğu."""

from __future__ import annotations

import sys
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from threading import Thread
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.data_contracts import (
    AttitudeRad,
    BoundingBoxXYXY,
    CameraIntrinsics,
    Detection2D,
    FlightMode,
    GeoPointWGS84,
    MissionPacket,
    MissionState,
    NEDVector,
    SafetyGate,
    TargetClass,
    TelemetryPacket,
)
from comms_gateway.referee_client import (
    LocalOutboxQueue,
    RefereeClient,
    RefereeEndpoints,
    telemetry_to_referee_json,
)
from vision_engine.detector import (
    DEFAULT_DEBOUNCE_FRAMES,
    DetectionDebouncer,
    FrameCapture,
    VisionDetector,
    utc_iso8601_from_ns,
    utc_timestamp_ns,
)


class _FakeBoxes:
    def __init__(self, xyxy: np.ndarray, conf: np.ndarray, cls: np.ndarray) -> None:
        self.xyxy = xyxy
        self.conf = conf
        self.cls = cls

    def __len__(self) -> int:
        return len(self.xyxy)


class _FakeResult:
    def __init__(self, boxes: _FakeBoxes) -> None:
        self.boxes = boxes
        self.names = {0: "hss", 1: "vehicle"}


class FakeYolo:
    """Ultralytics yerine deterministik kutu üretici."""

    def __init__(self, miss_first: int = 0) -> None:
        self.calls = 0
        self.miss_first = miss_first

    def __call__(self, source: Any, **kwargs: Any) -> List[_FakeResult]:
        self.calls += 1
        if self.calls <= self.miss_first:
            empty = _FakeBoxes(
                np.zeros((0, 4)), np.zeros((0,)), np.zeros((0,))
            )
            return [_FakeResult(empty)]
        xyxy = np.array([[100.0, 120.0, 220.0, 260.0]], dtype=np.float64)
        conf = np.array([0.92], dtype=np.float64)
        cls = np.array([0.0], dtype=np.float64)
        # hafif jitter
        xyxy[0, 0] += (self.calls % 3) * 2.0
        return [_FakeResult(_FakeBoxes(xyxy, conf, cls))]


class FakeCapture:
    def __init__(self, frames: int = 30) -> None:
        self.frames = frames
        self.i = 0
        self._opened = True

    def isOpened(self) -> bool:
        return self._opened

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        if self.i >= self.frames:
            return False, None
        self.i += 1
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[:, :] = (40, 40, 40)
        return True, img

    def get(self, prop: int) -> float:
        return 0.0

    def release(self) -> None:
        self._opened = False


def test_utc_timestamp_and_debounce_lock() -> None:
    t0 = utc_timestamp_ns()
    assert t0 > 0
    iso = utc_iso8601_from_ns(t0)
    assert iso.endswith("Z")

    deb = DetectionDebouncer(required_frames=8, iou_threshold=0.3, conf_threshold=0.5)
    box = BoundingBoxXYXY(100, 100, 200, 200)
    for i in range(7):
        det = Detection2D(0, 0, TargetClass.HSS, 0.9, box)
        st = deb.update([det])
        assert not st.locked
        assert st.streak == i + 1
    st = deb.update([Detection2D(0, 0, TargetClass.HSS, 0.9, box)])
    assert st.locked
    assert st.streak == 8
    assert DEFAULT_DEBOUNCE_FRAMES == 8


def test_vision_detector_eight_frame_debounce() -> None:
    cam = CameraIntrinsics(900, 900, 320, 240, 640, 480)
    det = VisionDetector(
        model=FakeYolo(),
        camera=cam,
        capture=FakeCapture(20),
        debounce_frames=8,
        model_name="fake-yolo",
        tensorrt_engine=False,
    )
    packets = []
    for _ in range(8):
        packets.append(det.step(sync_telemetry_seq=1))
    assert packets[0].timestamp_ns > 0
    assert packets[-1].frame_id == 8
    assert det.debounce_status.locked
    assert packets[-1].detections[0].class_name == TargetClass.HSS
    ann = det.annotated_frame()
    assert ann is not None and ann.shape == (480, 640, 3)
    det.close()


def test_outbox_queue_persists_json_and_blob(tmp_path: Path) -> None:
    q = LocalOutboxQueue(tmp_path / "outbox")
    from comms_gateway.referee_client import QueueItemType

    item_id = q.enqueue(
        QueueItemType.TELEMETRY,
        "/api/telemetry",
        {"hello": 1},
        blob=b"\xff\xd8fakejpeg",
    )
    assert q.size() == 1
    rows = q.peek_batch(4)
    assert rows[0]["id"] == item_id
    payload, blob = q.load_payload(rows[0])
    assert payload["hello"] == 1
    assert blob == b"\xff\xd8fakejpeg"
    q.ack(item_id)
    assert q.size() == 0


class _Recorder(BaseHTTPRequestHandler):
    store: List[Dict[str, Any]] = []

    def do_POST(self) -> None:  # noqa: N802
        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length)
        ctype = self.headers.get("Content-Type", "")
        _Recorder.store.append(
            {"path": self.path, "ctype": ctype, "body": body, "len": length}
        )
        self.send_response(200)
        self.end_headers()
        self.wfile.write(b'{"ok":true}')

    def log_message(self, format: str, *args: Any) -> None:  # noqa: A003
        return


def _telem() -> TelemetryPacket:
    return TelemetryPacket(
        timestamp_ns=time.time_ns(),
        sequence=7,
        position=GeoPointWGS84(39.92, 32.85, 950.0),
        relative_alt_m=80.0,
        attitude=AttitudeRad(0.01, -0.01, 0.0),
        velocity_ned_mps=NEDVector(20.0, 0.0, 0.0),
        airspeed_mps=22.0,
        groundspeed_mps=20.0,
        heading_deg=0.0,
        flight_mode=FlightMode.GUIDED,
        armed=True,
        gps_fix_num=3,
        satellites_visible=10,
        battery_voltage_v=22.0,
        battery_remaining_pct=70.0,
    )


def test_referee_offline_queue_then_flush(tmp_path: Path) -> None:
    # Önce erişilemeyen endpoint → kuyruk
    client = RefereeClient(
        RefereeEndpoints("http://127.0.0.1:1", timeout_s=0.3),
        queue_dir=tmp_path / "q1",
        telemetry_min_interval_s=0.0,
    )
    ok = client.push_telemetry(_telem(), force=True)
    assert ok is False
    assert client.pending_count >= 1
    assert client.is_online is False

    frame = np.zeros((120, 160, 3), dtype=np.uint8)
    dets = (
        Detection2D(
            0, 0, TargetClass.HSS, 0.88, BoundingBoxXYXY(10, 10, 50, 60)
        ),
    )
    ok_photo = client.submit_lock_photo(frame, dets, _telem())
    assert ok_photo is False
    pending_before = client.pending_count
    assert pending_before >= 2
    client.close()

    # Canlı sahte sunucu
    _Recorder.store.clear()
    server = HTTPServer(("127.0.0.1", 0), _Recorder)
    port = server.server_address[1]
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()

    client2 = RefereeClient(
        RefereeEndpoints(f"http://127.0.0.1:{port}", timeout_s=2.0),
        queue_dir=tmp_path / "q1",
        telemetry_min_interval_s=0.0,
    )
    # Aynı outbox dizinini kullan — flush önceki öğeleri gönderir
    sent = client2.flush_outbox(max_items=16)
    assert sent >= 1
    assert len(_Recorder.store) >= 1

    # Online telemetri
    assert client2.push_telemetry(_telem(), force=True) is True
    mission = MissionPacket(
        timestamp_ns=time.time_ns(),
        sequence=1,
        state=MissionState.TRACK_LOCK,
        previous_state=MissionState.SEARCH,
        safety=SafetyGate(
            True,
            True,
            True,
            False,
            1.0,
            -0.5,
            telemetry_fresh=True,
            altitude_ok=True,
        ),
        active_target=GeoPointWGS84(39.93, 32.85, 900.0),
        setpoint=None,
        message="locked",
    )
    payload = telemetry_to_referee_json(_telem(), mission)
    assert payload["mission"]["safety"]["telemetry_fresh"] is True
    assert payload["mission"]["safety"]["altitude_ok"] is True
    assert payload["position"]["agl_altitude_m"] is None
    assert client2.submit_release_photo(frame, dets, _telem(), mission) is True
    assert any(r["path"] == "/api/photo" for r in _Recorder.store)

    client2.close()
    server.shutdown()
