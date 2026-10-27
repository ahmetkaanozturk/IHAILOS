"""estimator: geo-referencing, balistik, 2D Kalman — sentetik doğrulama."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.data_contracts import AttitudeRad, CameraIntrinsics, GeoPointWGS84
from estimator.ballistics import (
    FAW_STANDOFF_M,
    freefall_time_s,
    haversine_distance_m,
    plan_ballistic_engagement,
    predict_impact_from_release,
)
from estimator.geo_referencing import (
    DEFAULT_PITCH_MOUNT_DEG,
    camera_intrinsics_matrix,
    pixel_to_wgs84,
)
from estimator.kalman_tracker import TargetKalmanTracker2D


def _camera() -> CameraIntrinsics:
    return CameraIntrinsics(
        fx=900.0,
        fy=900.0,
        cx=640.0,
        cy=360.0,
        width_px=1280,
        height_px=720,
    )


def test_camera_k_matrix() -> None:
    k = camera_intrinsics_matrix(_camera())
    assert k.shape == (3, 3)
    assert k[0, 0] == 900.0
    assert k[1, 1] == 900.0
    assert k[0, 2] == 640.0
    assert k[2, 2] == 1.0


def test_nadirish_pixel_projects_ahead_with_30deg_mount() -> None:
    """Level flight + 30° mount: görüntü merkezi burnun önünde yere düşer."""
    uav = GeoPointWGS84(39.92077, 32.85411, 1000.0)
    attitude = AttitudeRad(roll=0.0, pitch=0.0, yaw=0.0)  # kuzeye bakış
    agl = 100.0
    result = pixel_to_wgs84(
        u=640.0,
        v=360.0,
        camera=_camera(),
        attitude=attitude,
        uav_position=uav,
        lidar_agl_m=agl,
        pitch_mount_deg=DEFAULT_PITCH_MOUNT_DEG,
    )
    # Optik eksen 30° aşağı: yatay menzil = h / tan(30°) = 100*sqrt(3) ≈ 173.2 m kuzey
    assert result.north_m == pytest.approx(agl / math.tan(math.radians(30.0)), rel=0.05)
    assert abs(result.east_m) < 5.0
    assert result.down_m == pytest.approx(agl, rel=1e-6)
    assert result.target_wgs84.latitude_deg > uav.latitude_deg


def test_freefall_and_release_faw_geometry() -> None:
    h = 80.0
    t = freefall_time_s(h)
    assert t == pytest.approx(math.sqrt(2.0 * h / 9.80665), rel=1e-9)

    target = GeoPointWGS84(39.92, 32.85, 900.0)
    # Kuzeye 25 m/s yer hızı, rüzgarsız (hava=yer)
    plan = plan_ballistic_engagement(
        target=target,
        airspeed_mps=25.0,
        heading_deg=0.0,
        groundspeed_north_mps=25.0,
        groundspeed_east_mps=0.0,
        height_agl_m=h,
    )
    assert plan.time_of_flight_s == pytest.approx(t, rel=1e-9)
    assert plan.wind.speed_mps == pytest.approx(0.0, abs=1e-9)

    # Release, hedefin güneyinde (ters yer hızı)
    assert plan.release_point.latitude_deg < target.latitude_deg
    sep = haversine_distance_m(plan.final_approach_waypoint, plan.release_point)
    assert sep == pytest.approx(FAW_STANDOFF_M, rel=0.02)

    # İleri model: release'ten impact ≈ target
    impact = predict_impact_from_release(
        plan.release_point,
        groundspeed_north_mps=25.0,
        groundspeed_east_mps=0.0,
        height_agl_m=h,
    )
    assert haversine_distance_m(impact, target) < 3.0


def test_wind_from_pitot_gps_difference() -> None:
    target = GeoPointWGS84(39.92, 32.85, 900.0)
    # Hava kuzeye 30, yer kuzeye 25 → rüzgar güneye 5 (from north)
    plan = plan_ballistic_engagement(
        target=target,
        airspeed_mps=30.0,
        heading_deg=0.0,
        groundspeed_north_mps=25.0,
        groundspeed_east_mps=0.0,
        height_agl_m=50.0,
    )
    assert plan.wind.north_mps == pytest.approx(-5.0, abs=1e-9)
    assert plan.wind.east_mps == pytest.approx(0.0, abs=1e-9)
    assert plan.wind_drift_north_m == pytest.approx(
        -5.0 * plan.time_of_flight_s, rel=1e-9
    )


def test_kalman_damps_jumping_coordinates() -> None:
    origin = GeoPointWGS84(39.9, 32.8, 0.0)
    tracker = TargetKalmanTracker2D(
        origin_wgs84=origin,
        dt_s=0.1,
        process_var=1.0,
        measurement_var=40.0,
        gate_chi2=25.0,
        converge_updates=5,
    )
    rng = np.random.default_rng(0)
    true_n, true_e = 100.0, -40.0
    noises = []
    filt_errs = []
    for i in range(40):
        # Sabit hedef + sıçrayan gürültü
        n_meas = true_n + float(rng.normal(0.0, 8.0))
        e_meas = true_e + float(rng.normal(0.0, 8.0))
        if i == 20:
            n_meas += 80.0  # tek frame outlier
        track = tracker.update_ned(n_meas, e_meas, alt_m=0.0)
        noises.append(math.hypot(n_meas - true_n, e_meas - true_e))
        filt_errs.append(math.hypot(track.north_m - true_n, track.east_m - true_e))

    assert tracker.update_count >= 5
    assert track.filter_converged
    # Filtre RMSE, ham ölçüm RMSE'den düşük olmalı
    assert float(np.mean(filt_errs[10:])) < float(np.mean(noises[10:]))
    cov = tracker.position_covariance_3x3()
    assert len(cov) == 9
    assert cov[0] > 0.0 and cov[4] > 0.0
