"""
IHAILOS — çok iş parçacıklı ana döngü.

İş parçacıkları:
  1) telemetry  — MAVLink / sentetik TelemetryPacket → ring-buffer
  2) vision     — OpenCV + YOLO (+ debounce) → VisionPacket
  3) estimator  — geo-ref + Kalman + rüzgar/balistik → EstimatorPacket
  4) mission    — FSM + geofence + güvenlik → MissionPacket / MAVLink
  5) comms      — hakem JSON telemetri + outbox flush

Kullanım:
  python main.py --synthetic --duration 8
  python main.py --mavlink udp:127.0.0.1:14550 --camera 0 --model yolov8n.pt
"""

from __future__ import annotations

import argparse
import logging
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.data_contracts import (
    AttitudeRad,
    Detection2D,
    EstimatorPacket,
    FlightMode,
    GeoPointWGS84,
    MissionPacket,
    MissionState,
    NEDVector,
    TelemetryPacket,
    VisionPacket,
)
from comms_gateway.referee_client import RefereeClient, RefereeEndpoints
from estimator.ballistics import plan_ballistic_engagement
from estimator.geo_referencing import bbox_center_to_wgs84
from estimator.kalman_tracker import TargetKalmanTracker2D
from mission_coordinator.geofence import MissionGeofence
from mission_coordinator.mavlink_interface import MavlinkInterface
from mission_coordinator.state_machine import MissionConfig, MissionStateMachine
from vision_engine.detector import VisionDetector, default_intrinsics

LOGGER = logging.getLogger("ihailos.main")

# Sentetik operasyon poligonu (lon, lat)
DEFAULT_OPS = (
    (32.84, 39.91),
    (32.88, 39.91),
    (32.88, 39.95),
    (32.84, 39.95),
)


@dataclass
class SharedBus:
    """Modüller arası kilitli son-durum otobüsü."""

    lock: threading.RLock = field(default_factory=threading.RLock)
    stop: threading.Event = field(default_factory=threading.Event)
    telemetry: Optional[TelemetryPacket] = None
    vision: Optional[VisionPacket] = None
    estimator: Optional[EstimatorPacket] = None
    mission: Optional[MissionPacket] = None
    last_frame_bgr: Optional[np.ndarray] = None
    vision_detections: Tuple[Detection2D, ...] = ()
    ticks: dict = field(default_factory=lambda: {
        "telemetry": 0,
        "vision": 0,
        "estimator": 0,
        "mission": 0,
        "comms": 0,
    })

    def set_telemetry(self, pkt: TelemetryPacket) -> None:
        with self.lock:
            self.telemetry = pkt
            self.ticks["telemetry"] += 1

    def set_vision(
        self,
        pkt: VisionPacket,
        frame_bgr: Optional[np.ndarray],
        detections: Tuple[Detection2D, ...],
    ) -> None:
        with self.lock:
            self.vision = pkt
            self.last_frame_bgr = None if frame_bgr is None else frame_bgr.copy()
            self.vision_detections = detections
            self.ticks["vision"] += 1

    def set_estimator(self, pkt: EstimatorPacket) -> None:
        with self.lock:
            self.estimator = pkt
            self.ticks["estimator"] += 1

    def set_mission(self, pkt: MissionPacket) -> None:
        with self.lock:
            self.mission = pkt
            self.ticks["mission"] += 1

    def snapshot(
        self,
    ) -> Tuple[
        Optional[TelemetryPacket],
        Optional[VisionPacket],
        Optional[EstimatorPacket],
        Optional[MissionPacket],
        Optional[np.ndarray],
        Tuple[Detection2D, ...],
    ]:
        with self.lock:
            frame = None if self.last_frame_bgr is None else self.last_frame_bgr.copy()
            return (
                self.telemetry,
                self.vision,
                self.estimator,
                self.mission,
                frame,
                self.vision_detections,
            )


def _synthetic_telemetry(seq: int, t0: float) -> TelemetryPacket:
    """Kuzeye doğru GUIDED uçuş — sentetik telemetri."""
    elapsed = time.monotonic() - t0
    timestamp_ns = time.time_ns()
    lat0, lon0 = 39.925, 32.855
    # ~25 m/s kuzey → derece ofset
    dlat = (25.0 * elapsed) / 111320.0
    return TelemetryPacket(
        timestamp_ns=timestamp_ns,
        sequence=seq,
        position=GeoPointWGS84(lat0 + dlat, lon0, 980.0),
        relative_alt_m=80.0,
        agl_altitude_m=80.0,
        agl_timestamp_ns=timestamp_ns,
        position_timestamp_ns=timestamp_ns,
        attitude_timestamp_ns=timestamp_ns,
        heartbeat_timestamp_ns=timestamp_ns,
        # roll/pitch ≈ 0: güvenlik kapısı ve lat/lon demosu için temiz NED
        attitude=AttitudeRad(roll=0.0, pitch=0.0, yaw=0.0),
        velocity_ned_mps=NEDVector(25.0, 0.0, 0.0),
        # Vg == Va ⇒ rüzgarsız sentetik taban (bilinçli rüzgar testi ayrı)
        airspeed_mps=25.0,
        groundspeed_mps=25.0,
        heading_deg=0.0,
        flight_mode=FlightMode.GUIDED,
        armed=True,
        gps_fix_num=3,
        satellites_visible=12,
        battery_voltage_v=22.2,
        battery_remaining_pct=max(20.0, 90.0 - elapsed * 0.05),
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
        self.names = {0: "hss"}


class SyntheticYolo:
    """Donanım yokken Ultralytics yerine sabit HSS kutusu."""

    def __call__(self, source, **kwargs):  # type: ignore[no-untyped-def]
        h, w = source.shape[:2]
        cx, cy = w * 0.5, h * 0.55
        xyxy = np.array([[cx - 40, cy - 50, cx + 40, cy + 50]], dtype=np.float64)
        conf = np.array([0.91], dtype=np.float64)
        cls = np.array([0.0], dtype=np.float64)
        return [_FakeResult(_FakeBoxes(xyxy, conf, cls))]


class SyntheticCapture:
    def __init__(self) -> None:
        self._opened = True

    def isOpened(self) -> bool:
        return self._opened

    def read(self):
        img = np.zeros((720, 1280, 3), dtype=np.uint8)
        img[:] = (35, 35, 35)
        return True, img

    def get(self, _prop: int) -> float:
        return 0.0

    def release(self) -> None:
        self._opened = False


class FakeMavMaster:
    """Sentetik modda pymavlink master yerine no-op."""

    def __init__(self) -> None:
        self.target_system = 1
        self.target_component = 1
        self.mav = self

    def command_long_send(self, *args) -> None:  # type: ignore[no-untyped-def]
        LOGGER.debug("MAV command_long %s", args[2] if len(args) > 2 else args)

    def set_position_target_global_int_send(self, *args) -> None:  # type: ignore[no-untyped-def]
        LOGGER.debug("MAV setpoint lat_e7=%s", args[5] if len(args) > 5 else None)

    def recv_match(self, blocking: bool = False, timeout: float = 0.0):
        return None

    def close(self) -> None:
        return None


def fuse_estimator(
    telem: TelemetryPacket,
    vision: VisionPacket,
    tracker: TargetKalmanTracker2D,
) -> Optional[EstimatorPacket]:
    """Vision bbox → WGS84 → Kalman → EstimatorPacket (+ balistik özet)."""
    best = vision.best_detection()
    if best is None or telem.agl_altitude_m is None:
        return None

    geo = bbox_center_to_wgs84(
        x1=best.bbox.x1,
        y1=best.bbox.y1,
        x2=best.bbox.x2,
        y2=best.bbox.y2,
        camera=vision.camera,
        attitude=telem.attitude,
        uav_position=telem.position,
        lidar_agl_m=max(telem.agl_altitude_m, 1.0),
    )
    tracked = tracker.update_wgs84(geo.target_wgs84, dt_s=0.05)
    plan = plan_ballistic_engagement(
        target=tracked.wgs84,
        airspeed_mps=telem.airspeed_mps,
        heading_deg=telem.heading_deg,
        groundspeed_north_mps=telem.velocity_ned_mps.north,
        groundspeed_east_mps=telem.velocity_ned_mps.east,
        height_agl_m=max(telem.agl_altitude_m, 1.0),
    )
    return EstimatorPacket(
        timestamp_ns=vision.timestamp_ns,
        source_vision_frame_id=vision.frame_id,
        source_telemetry_seq=telem.sequence,
        target_class=best.class_name,
        target_wgs84=tracked.wgs84,
        target_ned_from_uav=NEDVector(geo.north_m, geo.east_m, geo.down_m),
        position_covariance=tracker.position_covariance_3x3(),
        kalman_innovation_norm=tracked.innovation_norm,
        wind=plan.wind,
        ballistic=plan.solution,
        confidence=best.confidence,
        filter_converged=tracked.filter_converged,
        valid=True,
    )


def telemetry_worker(
    bus: SharedBus,
    mav: MavlinkInterface,
    synthetic: bool,
    rate_hz: float,
) -> None:
    period = 1.0 / rate_hz
    seq = 0
    t0 = time.monotonic()
    while not bus.stop.is_set():
        loop_start = time.monotonic()
        try:
            if synthetic:
                seq += 1
                pkt = _synthetic_telemetry(seq, t0)
                mav.push_telemetry(pkt)
                bus.set_telemetry(pkt)
            else:
                pkt = mav.spin_once(timeout_s=period)
                if pkt is not None:
                    bus.set_telemetry(pkt)
                else:
                    latest = mav.latest()
                    if latest is not None:
                        bus.set_telemetry(latest)
        except Exception:
            LOGGER.exception("telemetry_worker hata")
        elapsed = time.monotonic() - loop_start
        bus.stop.wait(max(0.0, period - elapsed))


def vision_worker(
    bus: SharedBus,
    detector: VisionDetector,
    rate_hz: float,
) -> None:
    period = 1.0 / rate_hz
    while not bus.stop.is_set():
        loop_start = time.monotonic()
        try:
            telem, *_ = bus.snapshot()
            sync_seq = 0 if telem is None else telem.sequence
            pkt = detector.step(sync_telemetry_seq=sync_seq)
            frame = detector.last_frame_bgr
            bus.set_vision(pkt, frame, pkt.detections)
        except Exception:
            LOGGER.exception("vision_worker hata")
        elapsed = time.monotonic() - loop_start
        bus.stop.wait(max(0.0, period - elapsed))


def estimator_worker(
    bus: SharedBus,
    origin: GeoPointWGS84,
    rate_hz: float,
) -> None:
    period = 1.0 / rate_hz
    tracker = TargetKalmanTracker2D(origin_wgs84=origin, dt_s=period)
    while not bus.stop.is_set():
        loop_start = time.monotonic()
        try:
            telem, vision, *_ = bus.snapshot()
            if telem is not None and vision is not None and vision.detection_count > 0:
                est = fuse_estimator(telem, vision, tracker)
                if est is not None:
                    bus.set_estimator(est)
        except Exception:
            LOGGER.exception("estimator_worker hata")
        elapsed = time.monotonic() - loop_start
        bus.stop.wait(max(0.0, period - elapsed))


def mission_worker(
    bus: SharedBus,
    fsm: MissionStateMachine,
    rate_hz: float,
) -> None:
    period = 1.0 / rate_hz
    prev_state: Optional[MissionState] = None
    while not bus.stop.is_set():
        loop_start = time.monotonic()
        try:
            telem, _vision, est, _mission, frame, dets = bus.snapshot()
            if telem is None:
                bus.stop.wait(period)
                continue
            pkt = fsm.tick(telem, est)
            bus.set_mission(pkt)
            # Durum kenarı: kilit / salım fotoğrafı için bayrak (comms okur)
            if prev_state is not None and pkt.state != prev_state:
                LOGGER.info("FSM %s → %s | %s", prev_state.value, pkt.state.value, pkt.message)
            prev_state = pkt.state
            _ = (frame, dets)
        except Exception:
            LOGGER.exception("mission_worker hata")
        elapsed = time.monotonic() - loop_start
        bus.stop.wait(max(0.0, period - elapsed))


def comms_worker(
    bus: SharedBus,
    referee: RefereeClient,
    rate_hz: float,
) -> None:
    period = 1.0 / rate_hz
    last_state: Optional[MissionState] = None
    while not bus.stop.is_set():
        loop_start = time.monotonic()
        try:
            telem, _v, _e, mission, frame, dets = bus.snapshot()
            if telem is not None:
                referee.push_telemetry(telem, mission)
            if mission is not None and mission.state != last_state:
                if mission.state in (MissionState.TRACK_LOCK, MissionState.RELEASE):
                    if frame is not None:
                        referee.notify_mission_edge(mission, telem or _synthetic_telemetry(0, time.monotonic()), frame, dets)
                    else:
                        referee.notify_mission_edge(
                            mission,
                            telem or _synthetic_telemetry(0, time.monotonic()),
                        )
                last_state = mission.state
            flushed = referee.flush_outbox(max_items=8)
            if flushed:
                LOGGER.info("outbox flush: %d öğe", flushed)
            with bus.lock:
                bus.ticks["comms"] += 1
        except Exception:
            LOGGER.exception("comms_worker hata")
        elapsed = time.monotonic() - loop_start
        bus.stop.wait(max(0.0, period - elapsed))


def build_runtime(args: argparse.Namespace) -> tuple:
    bus = SharedBus()
    camera = default_intrinsics(1280, 720)

    if args.synthetic:
        mav = MavlinkInterface(master=FakeMavMaster())
        detector = VisionDetector(
            model=SyntheticYolo(),
            camera=camera,
            capture=SyntheticCapture(),
            debounce_frames=args.debounce,
            model_name="synthetic-yolo",
            tensorrt_engine=False,
        )
    else:
        mav = MavlinkInterface(connection_string=args.mavlink)
        mav.connect(timeout_s=args.mavlink_timeout)
        if not args.model:
            raise SystemExit("--model YOLO ağırlık yolu zorunlu (sentetik değilse)")
        detector = VisionDetector.from_model_path(
            args.model,
            camera=camera,
            source=args.camera,
            debounce_frames=args.debounce,
        )
        detector.open()

    geofence = MissionGeofence(DEFAULT_OPS, corridor_half_width_m=args.corridor_m)
    fsm = MissionStateMachine(
        geofence,
        mav,
        MissionConfig(
            track_lock_updates=args.track_lock,
            corridor_half_width_m=args.corridor_m,
        ),
    )
    referee = RefereeClient(
        RefereeEndpoints(args.referee_url, timeout_s=args.referee_timeout),
        queue_dir=Path(args.outbox_dir),
        team_id=args.team_id,
        telemetry_min_interval_s=0.2,
    )
    referee.start_background_flusher(period_s=2.0)
    origin = GeoPointWGS84(39.925, 32.855, 980.0)
    return bus, mav, detector, fsm, referee, origin


def run(args: argparse.Namespace) -> int:
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s [%(threadName)s] %(message)s",
    )
    bus, mav, detector, fsm, referee, origin = build_runtime(args)

    threads = [
        threading.Thread(
            target=telemetry_worker,
            name="telemetry",
            args=(bus, mav, args.synthetic, args.telem_hz),
            daemon=True,
        ),
        threading.Thread(
            target=vision_worker,
            name="vision",
            args=(bus, detector, args.vision_hz),
            daemon=True,
        ),
        threading.Thread(
            target=estimator_worker,
            name="estimator",
            args=(bus, origin, args.est_hz),
            daemon=True,
        ),
        threading.Thread(
            target=mission_worker,
            name="mission",
            args=(bus, fsm, args.mission_hz),
            daemon=True,
        ),
        threading.Thread(
            target=comms_worker,
            name="comms",
            args=(bus, referee, args.comms_hz),
            daemon=True,
        ),
    ]

    LOGGER.info(
        "IHAILOS start synthetic=%s duration=%.1fs",
        args.synthetic,
        args.duration,
    )
    for th in threads:
        th.start()

    t_end = time.monotonic() + args.duration
    try:
        while time.monotonic() < t_end and not bus.stop.is_set():
            time.sleep(1.0)
            with bus.lock:
                ticks = dict(bus.ticks)
                state = None if bus.mission is None else bus.mission.state.value
                telem = bus.telemetry
                est = bus.estimator
            if telem is not None and est is not None:
                LOGGER.info(
                    "ticks=%s fsm=%s | UAV lat/lon=(%.6f, %.6f) TGT lat/lon=(%.6f, %.6f) "
                    "windN=%.2f pending_outbox=%d",
                    ticks,
                    state,
                    telem.position.latitude_deg,
                    telem.position.longitude_deg,
                    est.target_wgs84.latitude_deg,
                    est.target_wgs84.longitude_deg,
                    est.wind.north_mps,
                    referee.pending_count,
                )
            else:
                LOGGER.info(
                    "ticks=%s fsm=%s pending_outbox=%d (estimator henüz yok)",
                    ticks,
                    state,
                    referee.pending_count,
                )
    except KeyboardInterrupt:
        LOGGER.warning("KeyboardInterrupt — durduruluyor")
    finally:
        bus.stop.set()
        for th in threads:
            th.join(timeout=2.0)
        detector.close()
        mav.close()
        referee.close()
        LOGGER.info("IHAILOS durdu | son ticks=%s", bus.ticks)

    return 0


def parse_args(argv: Optional[list] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="IHAILOS multithreaded runner")
    p.add_argument("--synthetic", action="store_true", help="Kamera/FC olmadan sentetik koşu")
    p.add_argument("--duration", type=float, default=10.0, help="Saniye cinsinden süre")
    p.add_argument("--mavlink", type=str, default="udp:127.0.0.1:14550")
    p.add_argument("--mavlink-timeout", type=float, default=10.0)
    p.add_argument("--camera", type=str, default="0")
    p.add_argument("--model", type=str, default="")
    p.add_argument("--debounce", type=int, default=8)
    p.add_argument("--track-lock", type=int, default=5)
    p.add_argument("--corridor-m", type=float, default=75.0)
    p.add_argument("--referee-url", type=str, default="http://127.0.0.1:8080")
    p.add_argument("--referee-timeout", type=float, default=2.0)
    p.add_argument("--outbox-dir", type=str, default="data/outbox")
    p.add_argument("--team-id", type=str, default="IHAILOS")
    p.add_argument("--telem-hz", type=float, default=10.0)
    p.add_argument("--vision-hz", type=float, default=5.0)
    p.add_argument("--est-hz", type=float, default=10.0)
    p.add_argument("--mission-hz", type=float, default=10.0)
    p.add_argument("--comms-hz", type=float, default=5.0)
    p.add_argument("--verbose", action="store_true")
    args = p.parse_args(argv)
    # kamera indeksi int olabilir
    if isinstance(args.camera, str) and args.camera.isdigit():
        args.camera = int(args.camera)
    return args


def main() -> None:
    raise SystemExit(run(parse_args()))


if __name__ == "__main__":
    main()
