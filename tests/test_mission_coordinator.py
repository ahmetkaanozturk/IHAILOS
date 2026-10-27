"""mission_coordinator: geofence, mavlink ring-buffer/servo, FSM — sentetik."""

from __future__ import annotations

import math
import sys
import time
from dataclasses import replace
from types import SimpleNamespace
from pathlib import Path
from typing import Any, List

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.data_contracts import (
    AttitudeRad,
    EstimatorPacket,
    FlightMode,
    GeoPointWGS84,
    MavlinkSetpoint,
    MissionState,
    NEDVector,
    TargetClass,
    TelemetryPacket,
    WindEstimate,
)
from mission_coordinator.geofence import MissionGeofence
from mission_coordinator.mavlink_interface import (
    RELEASE_PWM_US,
    MavlinkInterface,
)
from mission_coordinator.state_machine import MissionConfig, MissionStateMachine
from pymavlink import mavutil


# Sentetik saha: Ankara civarı ~2 km kutu
_OPS = [
    (32.84, 39.91),
    (32.87, 39.91),
    (32.87, 39.94),
    (32.84, 39.94),
]


class FakeMav:
    """pymavlink master yerine komut kaydedici."""

    def __init__(self) -> None:
        self.target_system = 1
        self.target_component = 1
        self.commands: List[Any] = []
        self.mav = self

    def command_long_send(self, *args: Any) -> None:
        self.commands.append(("command_long", args))

    def set_position_target_global_int_send(self, *args: Any) -> None:
        self.commands.append(("set_position_target_global_int", args))

    def recv_match(self, blocking: bool = False, timeout: float = 0.0) -> None:
        return None

    def close(self) -> None:
        return None


def _telem(
    lat: float = 39.925,
    lon: float = 32.855,
    mode: FlightMode = FlightMode.GUIDED,
    roll_deg: float = 1.0,
    pitch_deg: float = -0.5,
    seq: int = 1,
    agl_altitude_m: float | None = 80.0,
    timestamp_ns: int | None = None,
) -> TelemetryPacket:
    packet_timestamp_ns = time.time_ns() if timestamp_ns is None else timestamp_ns
    return TelemetryPacket(
        timestamp_ns=packet_timestamp_ns,
        sequence=seq,
        position=GeoPointWGS84(lat, lon, 980.0),
        relative_alt_m=80.0,
        agl_altitude_m=agl_altitude_m,
        agl_timestamp_ns=(
            packet_timestamp_ns if agl_altitude_m is not None else None
        ),
        position_timestamp_ns=packet_timestamp_ns,
        attitude_timestamp_ns=packet_timestamp_ns,
        heartbeat_timestamp_ns=packet_timestamp_ns,
        attitude=AttitudeRad(
            roll=math.radians(roll_deg),
            pitch=math.radians(pitch_deg),
            yaw=0.0,
        ),
        velocity_ned_mps=NEDVector(25.0, 0.0, 0.0),
        airspeed_mps=25.0,
        groundspeed_mps=25.0,
        heading_deg=0.0,
        flight_mode=mode,
        armed=True,
        gps_fix_num=3,
        satellites_visible=12,
        battery_voltage_v=22.0,
        battery_remaining_pct=80.0,
    )


def _est(lat: float = 39.930, lon: float = 32.855, conf: float = 0.9) -> EstimatorPacket:
    return EstimatorPacket(
        timestamp_ns=time.time_ns(),
        source_vision_frame_id=1,
        source_telemetry_seq=1,
        target_class=TargetClass.HSS,
        target_wgs84=GeoPointWGS84(lat, lon, 900.0),
        target_ned_from_uav=NEDVector(500.0, 0.0, 80.0),
        position_covariance=(4.0, 0, 0, 0, 4.0, 0, 0, 0, 9.0),
        kalman_innovation_norm=0.5,
        wind=WindEstimate(0.0, 0.0, 0.0, 0.0, 0.0),
        ballistic=None,
        confidence=conf,
        filter_converged=True,
        valid=True,
    )


def test_geofence_target_and_corridor() -> None:
    gf = MissionGeofence(_OPS, corridor_half_width_m=80.0)
    inside = GeoPointWGS84(39.925, 32.855, 900.0)
    outside = GeoPointWGS84(39.900, 32.855, 900.0)
    assert gf.target_inside(inside)
    assert not gf.target_inside(outside)

    faw = GeoPointWGS84(39.920, 32.855, 980.0)
    release = GeoPointWGS84(39.927, 32.855, 980.0)
    target = GeoPointWGS84(39.930, 32.855, 900.0)
    gf.set_approach_corridor(faw, release, target)
    verdict = gf.evaluate(inside, target, (faw, release, target))
    assert verdict.ok_for_engage


def test_mavlink_ring_buffer_rc_override_and_servo() -> None:
    fake = FakeMav()
    mav = MavlinkInterface(master=fake, ring_size=8)
    safe = _telem(mode=FlightMode.GUIDED, seq=1)
    mav.push_telemetry(safe)
    assert mav.ring_len == 1
    assert not mav.is_rc_override_active()
    gate = mav.evaluate_safety()
    assert gate.flight_mode_ok and gate.attitude_ok
    assert not gate.all_clear

    unsafe = _telem(mode=FlightMode.FBWA, seq=2)
    mav.push_telemetry(unsafe)
    assert mav.is_rc_override_active()
    assert not mav.evaluate_safety().all_clear

    attitude_bad = _telem(mode=FlightMode.GUIDED, roll_deg=8.0, seq=3)
    mav.push_telemetry(attitude_bad)
    assert not mav.evaluate_safety().attitude_ok

    mav.send_guided_waypoint(
        MavlinkSetpoint(target=GeoPointWGS84(39.93, 32.855, 80.0))
    )
    mav.trigger_release_servo_aux9()
    kinds = [c[0] for c in fake.commands]
    assert "set_position_target_global_int" in kinds
    assert "command_long" in kinds
    servo_cmd = [c for c in fake.commands if c[0] == "command_long"][-1][1]
    # args: target_system, target_component, command, confirmation, p1..p7
    assert servo_cmd[2] == mavutil.mavlink.MAV_CMD_DO_SET_SERVO
    assert servo_cmd[4] == 9.0
    assert servo_cmd[5] == float(RELEASE_PWM_US)


def test_mavlink_uses_only_downward_distance_sensor_for_agl() -> None:
    mav = MavlinkInterface(master=FakeMav())
    mav._ingest_mavlink_message(
        SimpleNamespace(
            get_type=lambda: "ATTITUDE",
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
        )
    )
    mav._ingest_mavlink_message(
        SimpleNamespace(
            get_type=lambda: "DISTANCE_SENSOR",
            orientation=mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270,
            min_distance=20,
            max_distance=4000,
            current_distance=850,
        )
    )
    packet = mav._build_telemetry_packet(time.time_ns())
    assert packet.agl_altitude_m == 8.5
    assert packet.agl_timestamp_ns is not None

    mav._ingest_mavlink_message(
        SimpleNamespace(
            get_type=lambda: "DISTANCE_SENSOR",
            orientation=mavutil.mavlink.MAV_SENSOR_ROTATION_NONE,
            min_distance=20,
            max_distance=4000,
            current_distance=500,
        )
    )
    assert mav._build_telemetry_packet(time.time_ns()).agl_altitude_m == 8.5


def test_fsm_happy_path_to_release_and_rtl() -> None:
    gf = MissionGeofence(_OPS, corridor_half_width_m=120.0)
    fake = FakeMav()
    mav = MavlinkInterface(master=fake)
    fsm = MissionStateMachine(
        gf,
        mav,
        MissionConfig(
            track_lock_updates=3,
            release_radius_m=5000.0,  # sentetik: hemen RELEASE'e gir
            faw_accept_radius_m=5000.0,
            verify_hold_s=0.0,
        ),
    )

    # Hedef: UAV'nin kuzeyinde, poligon içinde
    est = _est(39.928, 32.855)
    pkt = None
    for i in range(3):
        pkt = fsm.tick(_telem(39.925, 32.855, seq=i + 1), est)
    assert fsm.state in (MissionState.TRACK_LOCK, MissionState.APPROACH_PLAN)

    # APPROACH_PLAN + FINAL_RUN + RELEASE
    for i in range(6):
        # FINAL_RUN'da release yakınlığı için UAV'yi release noktasına taşı
        if fsm._plan is not None:
            rp = fsm._plan.release_point
            telem = _telem(rp.latitude_deg, rp.longitude_deg, seq=10 + i)
        else:
            telem = _telem(39.925, 32.855, seq=10 + i)
        pkt = fsm.tick(telem, est)

    assert fsm._release_fired or fsm.state in (
        MissionState.RELEASE,
        MissionState.VERIFY,
        MissionState.RTL,
    )
    # Servo tetiklenmiş olmalı
    servo_cmds = [
        c
        for c in fake.commands
        if c[0] == "command_long" and c[1][2] == mavutil.mavlink.MAV_CMD_DO_SET_SERVO
    ]
    assert len(servo_cmds) >= 1

    # VERIFY süresi 0 → RTL
    for i in range(3):
        pkt = fsm.tick(_telem(39.925, 32.855, seq=50 + i), est)
    assert fsm.state == MissionState.RTL
    assert pkt is not None
    assert pkt.state == MissionState.RTL


def test_fsm_go_around_on_rc_override() -> None:
    gf = MissionGeofence(_OPS, corridor_half_width_m=120.0)
    mav = MavlinkInterface(master=FakeMav())
    fsm = MissionStateMachine(
        gf, mav, MissionConfig(track_lock_updates=2, go_around_reenter_search=True)
    )
    est = _est()
    fsm.tick(_telem(mode=FlightMode.GUIDED, seq=1), est)
    assert fsm.state == MissionState.TRACK_LOCK
    # RC override (FBWA) — salım/yaklaşma engellenir, GO_AROUND → SEARCH
    out = fsm.tick(_telem(mode=FlightMode.FBWA, seq=2), est)
    assert out.state in (MissionState.GO_AROUND, MissionState.SEARCH)
    assert out.safety.rc_override_active


def test_fsm_blocks_engagement_without_fresh_safe_agl_or_telemetry() -> None:
    stale_agl = replace(
        _telem(),
        agl_timestamp_ns=time.time_ns() - 2_000_000_000,
    )
    stale_attitude = replace(
        _telem(),
        attitude_timestamp_ns=time.time_ns() - 2_000_000_000,
    )
    cases = (
        _telem(agl_altitude_m=None),
        _telem(agl_altitude_m=120.1),
        _telem(timestamp_ns=time.time_ns() - 2_000_000_000),
        stale_agl,
        stale_attitude,
    )
    for telem in cases:
        fake = FakeMav()
        fsm = MissionStateMachine(
            MissionGeofence(_OPS, corridor_half_width_m=120.0),
            MavlinkInterface(master=fake),
        )
        out = fsm.tick(telem, _est())
        assert out.state == MissionState.SEARCH
        assert not out.engage_authorized
        assert not out.safety.all_clear
        assert not any(
            command[0] == "command_long"
            and command[1][2] == mavutil.mavlink.MAV_CMD_DO_SET_SERVO
            for command in fake.commands
        )
