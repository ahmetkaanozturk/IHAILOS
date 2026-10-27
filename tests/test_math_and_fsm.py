"""
Sentetik doğrulama: Geo-referencing, balistik rüzgar, Geofence (+ FSM güvenlik kenarı).
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.data_contracts import (
    AttitudeRad,
    CameraIntrinsics,
    FlightMode,
    GeoPointWGS84,
    NEDVector,
    TelemetryPacket,
)
from estimator.ballistics import (
    FAW_STANDOFF_M,
    estimate_wind_vector,
    freefall_time_s,
    haversine_distance_m,
    plan_ballistic_engagement,
    predict_impact_from_release,
    wind_drift_ned_m,
)
from estimator.geo_referencing import (
    DEFAULT_PITCH_MOUNT_DEG,
    body_to_ned_rotation,
    camera_intrinsics_matrix,
    meters_per_deg_lat,
    pixel_to_wgs84,
)
from shapely.geometry import Point

from mission_coordinator.geofence import MissionGeofence, corridor_polygon_wgs84
from mission_coordinator.mavlink_interface import MavlinkInterface
from mission_coordinator.state_machine import MissionConfig, MissionStateMachine
from common.data_contracts import EstimatorPacket, MissionState, TargetClass, WindEstimate


# ---------------------------------------------------------------------------
# Geo-referencing
# ---------------------------------------------------------------------------


def test_geo_pinhole_k_and_30deg_mount_ground_intersect() -> None:
    camera = CameraIntrinsics(
        fx=1000.0, fy=1000.0, cx=640.0, cy=360.0, width_px=1280, height_px=720
    )
    k = camera_intrinsics_matrix(camera)
    assert k[0, 0] == 1000.0 and k[1, 1] == 1000.0
    assert k[0, 2] == 640.0 and k[1, 2] == 360.0

    uav = GeoPointWGS84(39.9200, 32.8500, 1100.0)
    attitude = AttitudeRad(roll=0.0, pitch=0.0, yaw=0.0)  # kuzey
    agl = 120.0

    result = pixel_to_wgs84(
        u=640.0,
        v=360.0,
        camera=camera,
        attitude=attitude,
        uav_position=uav,
        lidar_agl_m=agl,
        pitch_mount_deg=DEFAULT_PITCH_MOUNT_DEG,
    )

    # Optik eksen 30° aşağı → yatay menzil = h / tan(30°)
    expected_north = agl / math.tan(math.radians(30.0))
    assert result.north_m == pytest.approx(expected_north, rel=0.03)
    assert abs(result.east_m) < 2.0
    assert result.down_m == pytest.approx(agl, abs=1e-6)
    assert result.target_wgs84.latitude_deg > uav.latitude_deg

    # NED rotasyon dikgen olmalı
    r = body_to_ned_rotation(attitude)
    assert np.allclose(r @ r.T, np.eye(3), atol=1e-9)


def test_geo_roll_shifts_east_component() -> None:
    camera = CameraIntrinsics(900.0, 900.0, 640.0, 360.0, 1280, 720)
    uav = GeoPointWGS84(40.0, 33.0, 1000.0)
    agl = 80.0
    level = pixel_to_wgs84(
        640, 360, camera, AttitudeRad(0.0, 0.0, 0.0), uav, agl
    )
    rolled = pixel_to_wgs84(
        640, 360, camera, AttitudeRad(math.radians(8.0), 0.0, 0.0), uav, agl
    )
    # Roll, yatay kesişimde doğu bileşeni üretir (FRD: +roll → −east)
    assert abs(rolled.east_m) > 5.0
    assert abs(level.east_m) < 1e-6
    assert rolled.east_m < 0.0


# ---------------------------------------------------------------------------
# Balistik / rüzgar
# ---------------------------------------------------------------------------


def test_ballistic_wind_from_pitot_minus_gps() -> None:
    # Va = 30 m/s kuzey, Vg = 25 kuzey + 4 doğu → Vw = (-5, +4)
    wind = estimate_wind_vector(
        airspeed_mps=30.0,
        heading_deg=0.0,
        groundspeed_north_mps=25.0,
        groundspeed_east_mps=4.0,
    )
    assert wind.north_mps == pytest.approx(-5.0, abs=1e-9)
    assert wind.east_mps == pytest.approx(4.0, abs=1e-9)
    assert wind.speed_mps == pytest.approx(math.hypot(5.0, 4.0), rel=1e-9)

    h = 100.0
    t = freefall_time_s(h)
    assert t == pytest.approx(math.sqrt(2.0 * h / 9.80665), rel=1e-12)
    dn, de = wind_drift_ned_m(wind, t)
    assert dn == pytest.approx(-5.0 * t, rel=1e-12)
    assert de == pytest.approx(4.0 * t, rel=1e-12)


def test_ballistic_release_and_faw_300m() -> None:
    target = GeoPointWGS84(39.930, 32.855, 900.0)
    plan = plan_ballistic_engagement(
        target=target,
        airspeed_mps=25.0,
        heading_deg=0.0,
        groundspeed_north_mps=25.0,
        groundspeed_east_mps=0.0,
        height_agl_m=80.0,
    )
    assert plan.wind.speed_mps == pytest.approx(0.0, abs=1e-9)
    # Release hedefin güneyinde
    assert plan.release_point.latitude_deg < target.latitude_deg
    sep = haversine_distance_m(plan.final_approach_waypoint, plan.release_point)
    assert sep == pytest.approx(FAW_STANDOFF_M, rel=0.02)

    impact = predict_impact_from_release(
        plan.release_point, 25.0, 0.0, height_agl_m=80.0
    )
    assert haversine_distance_m(impact, target) < 5.0


# ---------------------------------------------------------------------------
# Geofence
# ---------------------------------------------------------------------------


_OPS = (
    (32.84, 39.91),
    (32.87, 39.91),
    (32.87, 39.94),
    (32.84, 39.94),
)


def test_geofence_target_inside_outside_and_corridor() -> None:
    gf = MissionGeofence(_OPS, corridor_half_width_m=60.0)
    inside = GeoPointWGS84(39.925, 32.855, 900.0)
    outside = GeoPointWGS84(39.900, 32.800, 900.0)
    assert gf.target_inside(inside) is True
    assert gf.target_inside(outside) is False
    assert gf.uav_inside_ops(inside) is True

    faw = GeoPointWGS84(39.920, 32.855, 980.0)
    release = GeoPointWGS84(39.926, 32.855, 980.0)
    target = GeoPointWGS84(39.930, 32.855, 900.0)
    poly = gf.set_approach_corridor(faw, release, target)
    assert poly.area > 0.0

    corridor = corridor_polygon_wgs84((faw, release, target), 60.0, 39.925)
    assert corridor.covers(Point(32.855, 39.926))

    verdict = gf.evaluate(inside, target, (faw, release, target))
    assert verdict.target_inside
    assert verdict.approach_inside
    assert verdict.uav_inside_ops
    assert verdict.ok_for_engage

    bad = gf.evaluate(outside, target, (faw, release, target))
    assert bad.ok_for_engage is False


# ---------------------------------------------------------------------------
# FSM güvenlik (math/geofence ile entegre kenar)
# ---------------------------------------------------------------------------


class _FakeMav:
    def __init__(self) -> None:
        self.target_system = 1
        self.target_component = 1
        self.mav = self

    def command_long_send(self, *args) -> None:
        return None

    def set_position_target_global_int_send(self, *args) -> None:
        return None

    def recv_match(self, **kwargs):
        return None

    def close(self) -> None:
        return None


def _telem(mode: FlightMode = FlightMode.GUIDED) -> TelemetryPacket:
    timestamp_ns = time.time_ns()
    return TelemetryPacket(
        timestamp_ns=timestamp_ns,
        sequence=1,
        position=GeoPointWGS84(39.925, 32.855, 980.0),
        relative_alt_m=80.0,
        agl_altitude_m=80.0,
        agl_timestamp_ns=timestamp_ns,
        position_timestamp_ns=timestamp_ns,
        attitude_timestamp_ns=timestamp_ns,
        heartbeat_timestamp_ns=timestamp_ns,
        attitude=AttitudeRad(0.01, -0.01, 0.0),
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


def _est() -> EstimatorPacket:
    return EstimatorPacket(
        timestamp_ns=time.time_ns(),
        source_vision_frame_id=1,
        source_telemetry_seq=1,
        target_class=TargetClass.HSS,
        target_wgs84=GeoPointWGS84(39.928, 32.855, 900.0),
        target_ned_from_uav=NEDVector(300.0, 0.0, 80.0),
        position_covariance=(4, 0, 0, 0, 4, 0, 0, 0, 9),
        kalman_innovation_norm=0.4,
        wind=WindEstimate(0.0, 0.0, 0.0, 0.0, 0.0),
        ballistic=None,
        confidence=0.9,
        filter_converged=True,
        valid=True,
    )


def test_fsm_geofence_search_then_rc_override_go_around() -> None:
    gf = MissionGeofence(_OPS, corridor_half_width_m=100.0)
    mav = MavlinkInterface(master=_FakeMav())
    fsm = MissionStateMachine(
        gf, mav, MissionConfig(track_lock_updates=2, go_around_reenter_search=True)
    )
    est = _est()
    # Geofence içi hedef → SEARCH→TRACK_LOCK
    out1 = fsm.tick(_telem(FlightMode.GUIDED), est)
    assert out1.state == MissionState.TRACK_LOCK
    assert out1.safety.geofence_ok

    # RC override
    out2 = fsm.tick(_telem(FlightMode.MANUAL), est)
    assert out2.safety.rc_override_active
    assert out2.state in (MissionState.GO_AROUND, MissionState.SEARCH)


def test_meters_per_deg_lat_positive() -> None:
    m = meters_per_deg_lat(39.9)
    assert 110000.0 < m < 112000.0
