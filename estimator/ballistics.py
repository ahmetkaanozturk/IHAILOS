"""
Balistik / rüzgar: pito–GPS farkı, serbest düşüş, Release Point ve FAW.

Bölüm 2 — Estimator / Wind + Ballistics:
  V_wind = V_ground − V_air
  t = sqrt(2 h / g)
  sürüklenme = V_wind * t
  salım ofseti ≈ V_ground * t  (yatay)
  FAW = Release Point'in yaklaşma istikametinde 300 m gerisi
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np

from common.data_contracts import (
    BallisticSolution,
    GeoPointWGS84,
    WindEstimate,
)
from estimator.geo_referencing import ned_offset_to_wgs84

G_MPS2 = 9.80665
FAW_STANDOFF_M = 300.0


@dataclass(frozen=True)
class HorizontalWind:
    """Yatay rüzgar vektörü (NED, m/s) — V_g − V_a."""

    north_mps: float
    east_mps: float

    @property
    def speed_mps(self) -> float:
        return float(np.hypot(self.north_mps, self.east_mps))

    @property
    def direction_from_deg(self) -> float:
        """Meteorolojik yön: rüzgarın geldiği yön (0=Kuzey, saat yönü)."""
        # esme yönü = atan2(east, north); geldiği yön = +180°
        to_deg = float(np.rad2deg(np.atan2(self.east_mps, self.north_mps))) % 360.0
        return (to_deg + 180.0) % 360.0

    def to_wind_estimate(self, variance: float = 0.0) -> WindEstimate:
        return WindEstimate(
            speed_mps=self.speed_mps,
            direction_deg=self.direction_from_deg,
            north_mps=self.north_mps,
            east_mps=self.east_mps,
            variance=variance,
        )


def air_velocity_ned_mps(
    airspeed_mps: float,
    heading_deg: float,
) -> Tuple[float, float]:
    """
    Sideslip yok varsayımı: pito hava hızı vektörü yatayda heading yönünde.
    Dönüş: (v_north, v_east) m/s.
    """
    if airspeed_mps < 0.0:
        raise ValueError("airspeed_mps negatif olamaz")
    if not np.isfinite(airspeed_mps) or not np.isfinite(heading_deg):
        raise ValueError("airspeed ve heading sonlu olmalı")
    yaw = np.deg2rad(heading_deg)
    vn = float(airspeed_mps * np.cos(yaw))
    ve = float(airspeed_mps * np.sin(yaw))
    return vn, ve


def estimate_wind_vector(
    airspeed_mps: float,
    heading_deg: float,
    groundspeed_north_mps: float,
    groundspeed_east_mps: float,
) -> HorizontalWind:
    """
    Rüzgar: V_w = V_g − V_a.

    V_a : pito hava hızı × heading (yatay)
    V_g : GPS yer hızı NED yatay bileşenleri
    """
    if not np.isfinite(groundspeed_north_mps) or not np.isfinite(groundspeed_east_mps):
        raise ValueError("yer hızı bileşenleri sonlu olmalı")
    va_n, va_e = air_velocity_ned_mps(airspeed_mps, heading_deg)
    wn = float(groundspeed_north_mps - va_n)
    we = float(groundspeed_east_mps - va_e)
    return HorizontalWind(north_mps=wn, east_mps=we)


def freefall_time_s(height_m: float, g: float = G_MPS2) -> float:
    """Serbest düşüş süresi: t = sqrt(2h/g)."""
    if height_m < 0.0:
        raise ValueError("height_m negatif olamaz")
    if g <= 0.0:
        raise ValueError("g pozitif olmalı")
    if height_m == 0.0:
        return 0.0
    return float(np.sqrt(2.0 * height_m / g))


def wind_drift_ned_m(wind: HorizontalWind, time_of_flight_s: float) -> Tuple[float, float]:
    """Rüzgar sürüklenmesi (m): Δ = V_wind * t."""
    if time_of_flight_s < 0.0:
        raise ValueError("time_of_flight_s negatif olamaz")
    dn = wind.north_mps * time_of_flight_s
    de = wind.east_mps * time_of_flight_s
    return float(dn), float(de)


def ballistic_horizontal_offset_ned_m(
    groundspeed_north_mps: float,
    groundspeed_east_mps: float,
    time_of_flight_s: float,
) -> Tuple[float, float]:
    """
    Yük, salım anındaki yer hızını korur (basit model):
    Δ_ned = V_ground_horizontal * t.
    Salım noktası = hedef − Δ (etki noktası hedef olsun diye).
    """
    if time_of_flight_s < 0.0:
        raise ValueError("time_of_flight_s negatif olamaz")
    return (
        float(groundspeed_north_mps * time_of_flight_s),
        float(groundspeed_east_mps * time_of_flight_s),
    )


def compute_release_point(
    target: GeoPointWGS84,
    groundspeed_north_mps: float,
    groundspeed_east_mps: float,
    height_agl_m: float,
    g: float = G_MPS2,
) -> Tuple[GeoPointWGS84, float, Tuple[float, float]]:
    """
    Bırakma Noktası (Release Point).

    t = sqrt(2h/g)
    offset = V_g * t
    release = target − offset  (NED → WGS84)
    """
    if height_agl_m <= 0.0:
        raise ValueError("height_agl_m pozitif olmalı")
    t = freefall_time_s(height_agl_m, g=g)
    dn, de = ballistic_horizontal_offset_ned_m(
        groundspeed_north_mps, groundspeed_east_mps, t
    )
    # Hedefe vurmak için salım, yer hızı vektörünün tersi yönde geriye alınır
    release = ned_offset_to_wgs84(target, north_m=-dn, east_m=-de, down_m=0.0)
    # Salım irtifası: hedef zemin + AGL (UAV AMSL yaklaşık)
    release = GeoPointWGS84(
        latitude_deg=release.latitude_deg,
        longitude_deg=release.longitude_deg,
        altitude_m=target.altitude_m + height_agl_m,
    )
    return release, t, (dn, de)


def compute_final_approach_waypoint(
    release_point: GeoPointWGS84,
    groundspeed_north_mps: float,
    groundspeed_east_mps: float,
    standoff_m: float = FAW_STANDOFF_M,
) -> GeoPointWGS84:
    """
    Final Approach Waypoint (FAW): Release Point'in 300 m gerisi.

    Yaklaşma ekseni = yatay yer hızı birim vektörü.
    FAW = release − standoff * unit(V_g_horizontal).
    """
    if standoff_m <= 0.0:
        raise ValueError("standoff_m pozitif olmalı")
    speed = float(np.hypot(groundspeed_north_mps, groundspeed_east_mps))
    if speed < 0.5:
        raise ValueError("FAW için yer hızı çok düşük (|Vg|<0.5 m/s)")
    unit_n = groundspeed_north_mps / speed
    unit_e = groundspeed_east_mps / speed
    # Geride: yaklaşma yönünün tersi
    faw = ned_offset_to_wgs84(
        release_point,
        north_m=-unit_n * standoff_m,
        east_m=-unit_e * standoff_m,
        down_m=0.0,
    )
    return GeoPointWGS84(
        latitude_deg=faw.latitude_deg,
        longitude_deg=faw.longitude_deg,
        altitude_m=release_point.altitude_m,
    )


@dataclass(frozen=True)
class BallisticPlan:
    """Release Point + FAW + rüzgar / sürüklenme özeti."""

    wind: WindEstimate
    time_of_flight_s: float
    wind_drift_north_m: float
    wind_drift_east_m: float
    release_point: GeoPointWGS84
    final_approach_waypoint: GeoPointWGS84
    solution: BallisticSolution


def plan_ballistic_engagement(
    target: GeoPointWGS84,
    airspeed_mps: float,
    heading_deg: float,
    groundspeed_north_mps: float,
    groundspeed_east_mps: float,
    height_agl_m: float,
    faw_standoff_m: float = FAW_STANDOFF_M,
    g: float = G_MPS2,
    wind_variance: float = 0.0,
) -> BallisticPlan:
    """
    Uçtan uca balistik plan: rüzgar → t → sürüklenme → Release → FAW.
    """
    wind_vec = estimate_wind_vector(
        airspeed_mps=airspeed_mps,
        heading_deg=heading_deg,
        groundspeed_north_mps=groundspeed_north_mps,
        groundspeed_east_mps=groundspeed_east_mps,
    )
    release, t, offset = compute_release_point(
        target=target,
        groundspeed_north_mps=groundspeed_north_mps,
        groundspeed_east_mps=groundspeed_east_mps,
        height_agl_m=height_agl_m,
        g=g,
    )
    drift_n, drift_e = wind_drift_ned_m(wind_vec, t)
    faw = compute_final_approach_waypoint(
        release_point=release,
        groundspeed_north_mps=groundspeed_north_mps,
        groundspeed_east_mps=groundspeed_east_mps,
        standoff_m=faw_standoff_m,
    )
    # horizontal_miss_m: salım–hedef yatay menzili (balistik ofset büyüklüğü)
    miss = float(np.hypot(offset[0], offset[1]))
    solution = BallisticSolution(
        impact_point=target,
        release_point=release,
        time_of_flight_s=t,
        horizontal_miss_m=miss,
        valid=True,
    )
    return BallisticPlan(
        wind=wind_vec.to_wind_estimate(variance=wind_variance),
        time_of_flight_s=t,
        wind_drift_north_m=drift_n,
        wind_drift_east_m=drift_e,
        release_point=release,
        final_approach_waypoint=faw,
        solution=solution,
    )


def haversine_distance_m(a: GeoPointWGS84, b: GeoPointWGS84) -> float:
    """İki WGS84 noktası arası büyük daire mesafesi (m) — test/doğrulama."""
    r = 6371000.0
    lat1 = np.deg2rad(a.latitude_deg)
    lat2 = np.deg2rad(b.latitude_deg)
    dlat = lat2 - lat1
    dlon = np.deg2rad(b.longitude_deg - a.longitude_deg)
    h = (
        np.sin(dlat / 2.0) ** 2
        + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2.0) ** 2
    )
    return float(2.0 * r * np.arcsin(np.sqrt(h)))


def faw_release_separation_m(plan: BallisticPlan) -> float:
    """FAW–Release mesafesi; nominal 300 m."""
    return haversine_distance_m(plan.final_approach_waypoint, plan.release_point)


def predict_impact_from_release(
    release_point: GeoPointWGS84,
    groundspeed_north_mps: float,
    groundspeed_east_mps: float,
    height_agl_m: float,
    g: float = G_MPS2,
) -> GeoPointWGS84:
    """
    Verilen salım noktasından ileri balistik etki noktası (doğrulama).
    impact = release_horizontal + V_g * t, irtifa = release − AGL.
    """
    t = freefall_time_s(height_agl_m, g=g)
    dn, de = ballistic_horizontal_offset_ned_m(
        groundspeed_north_mps, groundspeed_east_mps, t
    )
    impact = ned_offset_to_wgs84(release_point, north_m=dn, east_m=de, down_m=0.0)
    return GeoPointWGS84(
        latitude_deg=impact.latitude_deg,
        longitude_deg=impact.longitude_deg,
        altitude_m=release_point.altitude_m - height_agl_m,
    )
