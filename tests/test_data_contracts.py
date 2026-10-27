"""Data contract serileştirme ve güvenlik kapısı duman testi."""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.data_contracts import (
    AttitudeRad,
    BoundingBoxXYXY,
    CameraIntrinsics,
    Detection2D,
    EstimatorPacket,
    FlightMode,
    GeoPointWGS84,
    MissionPacket,
    MissionState,
    NEDVector,
    SafetyGate,
    TargetClass,
    TelemetryPacket,
    VisionPacket,
    WindEstimate,
)


def _telemetry() -> TelemetryPacket:
    return TelemetryPacket(
        timestamp_ns=1_000_000_000,
        sequence=42,
        position=GeoPointWGS84(39.9, 32.8, 950.0),
        relative_alt_m=120.0,
        agl_altitude_m=120.0,
        agl_timestamp_ns=1_000_000_000,
        position_timestamp_ns=1_000_000_000,
        attitude_timestamp_ns=1_000_000_000,
        heartbeat_timestamp_ns=1_000_000_000,
        attitude=AttitudeRad(roll=0.02, pitch=-0.01, yaw=1.2),
        velocity_ned_mps=NEDVector(25.0, 3.0, -0.5),
        airspeed_mps=28.0,
        groundspeed_mps=26.0,
        heading_deg=68.0,
        flight_mode=FlightMode.GUIDED,
        armed=True,
        gps_fix_num=3,
        satellites_visible=12,
        battery_voltage_v=22.1,
        battery_remaining_pct=78.0,
    )


def test_telemetry_roundtrip_and_attitude_gate() -> None:
    pkt = _telemetry()
    assert pkt.is_auto_or_guided()
    assert pkt.attitude_within_limits(5.0)
    restored = TelemetryPacket.from_dict(pkt.to_dict())
    assert restored.sequence == 42
    assert restored.flight_mode == FlightMode.GUIDED
    assert math.isclose(restored.position.latitude_deg, 39.9)


def test_vision_best_detection() -> None:
    cam = CameraIntrinsics(fx=900.0, fy=900.0, cx=640.0, cy=360.0, width_px=1280, height_px=720)
    det_a = Detection2D(
        detection_id=0,
        class_id=1,
        class_name=TargetClass.HSS,
        confidence=0.71,
        bbox=BoundingBoxXYXY(100, 100, 200, 220),
    )
    det_b = Detection2D(
        detection_id=1,
        class_id=1,
        class_name=TargetClass.HSS,
        confidence=0.91,
        bbox=BoundingBoxXYXY(400, 300, 520, 450),
    )
    vision = VisionPacket(
        timestamp_ns=1_000_000_100,
        frame_id=7,
        sync_telemetry_seq=42,
        image_width=1280,
        image_height=720,
        detections=(det_a, det_b),
        camera=cam,
        inference_latency_ms=12.5,
        model_name="yolov8n-hss",
    )
    best = vision.best_detection(TargetClass.HSS)
    assert best is not None
    assert best.confidence == 0.91
    assert VisionPacket.from_dict(vision.to_dict()).detection_count == 2


def test_estimator_and_mission_contracts() -> None:
    est = EstimatorPacket(
        timestamp_ns=1_000_000_200,
        source_vision_frame_id=7,
        source_telemetry_seq=42,
        target_class=TargetClass.HSS,
        target_wgs84=GeoPointWGS84(39.901, 32.805, 890.0),
        target_ned_from_uav=NEDVector(180.0, -40.0, 30.0),
        position_covariance=(4.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 9.0),
        kalman_innovation_norm=1.2,
        wind=WindEstimate(5.0, 270.0, 0.0, -5.0, 0.5),
        ballistic=None,
        confidence=0.88,
        filter_converged=True,
        valid=True,
    )
    assert EstimatorPacket.from_dict(est.to_dict()).valid

    safety = SafetyGate(
        flight_mode_ok=True,
        attitude_ok=True,
        geofence_ok=True,
        rc_override_active=False,
        roll_deg=1.1,
        pitch_deg=-0.5,
        telemetry_fresh=True,
        altitude_ok=True,
    )
    mission = MissionPacket(
        timestamp_ns=1_000_000_300,
        sequence=1,
        state=MissionState.TRACK_LOCK,
        previous_state=MissionState.SEARCH,
        safety=safety,
        active_target=est.target_wgs84,
        setpoint=None,
        engage_authorized=True,
        message="track_locked",
    )
    assert mission.safety.all_clear
    assert MissionPacket.from_dict(mission.to_dict()).state == MissionState.TRACK_LOCK
