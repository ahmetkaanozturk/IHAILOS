"""
2D Kalman hedef izleyici — zıplayan geo-referencing çıktısını sabitler.

Bölüm 2 — Estimator / Kalman:
  Durum: [n, e, vn, ve]  (yerel NED metre + m/s)
  Ölçüm: [n, e]
  filterpy.kalman.KalmanFilter sabit hız modeli
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from common.data_contracts import GeoPointWGS84
from estimator.geo_referencing import (
    meters_per_deg_lat,
    meters_per_deg_lon,
    ned_offset_to_wgs84,
)


@dataclass(frozen=True)
class TrackedTarget2D:
    """Filtrelenmiş hedef durumu (NED + WGS84)."""

    north_m: float
    east_m: float
    vel_north_mps: float
    vel_east_mps: float
    wgs84: GeoPointWGS84
    innovation_norm: float
    covariance_ne: Tuple[float, float, float, float]  # 2x2 satır-major
    filter_converged: bool
    update_count: int


class TargetKalmanTracker2D:
    """
    Zıplayan (u,v)→WGS84 ölçümlerini 2D sabit-hız Kalman ile yumuşatır.

    Referans orijin: ilk geçerli ölçümün UAV/yerel teğet düzlemi köşesi
    (origin_wgs84). Tüm NED metrikleri bu orijine göredir.
    """

    def __init__(
        self,
        origin_wgs84: GeoPointWGS84,
        dt_s: float = 0.05,
        process_var: float = 2.0,
        measurement_var: float = 25.0,
        gate_chi2: float = 9.21,
        converge_updates: int = 8,
    ) -> None:
        if dt_s <= 0.0:
            raise ValueError("dt_s pozitif olmalı")
        if process_var < 0.0 or measurement_var < 0.0:
            raise ValueError("varyanslar negatif olamaz")
        if gate_chi2 <= 0.0:
            raise ValueError("gate_chi2 pozitif olmalı")
        if converge_updates < 1:
            raise ValueError("converge_updates >= 1 olmalı")

        self.origin_wgs84 = origin_wgs84
        self.dt_s = float(dt_s)
        self.process_var = float(process_var)
        self.measurement_var = float(measurement_var)
        self.gate_chi2 = float(gate_chi2)
        self.converge_updates = int(converge_updates)

        self._kf = self._build_filter()
        self._initialized = False
        self._update_count = 0
        self._last_innovation_norm = 0.0

    def _build_filter(self) -> KalmanFilter:
        kf = KalmanFilter(dim_x=4, dim_z=2)
        dt = self.dt_s
        # x = [n, e, vn, ve]^T
        kf.F = np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        kf.H = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
            ],
            dtype=np.float64,
        )
        # Q_discrete_white_noise [pos, vel] üretir; durum [n, e, vn, ve].
        # n↔vn indeksleri (0,2), e↔ve indeksleri (1,3).
        q_n = Q_discrete_white_noise(dim=2, dt=dt, var=self.process_var)
        q_e = Q_discrete_white_noise(dim=2, dt=dt, var=self.process_var)
        kf.Q = np.zeros((4, 4), dtype=np.float64)
        for src, (i, j) in ((q_n, (0, 2)), (q_e, (1, 3))):
            kf.Q[i, i] = src[0, 0]
            kf.Q[i, j] = src[0, 1]
            kf.Q[j, i] = src[1, 0]
            kf.Q[j, j] = src[1, 1]

        kf.R = np.eye(2, dtype=np.float64) * self.measurement_var
        kf.P = np.eye(4, dtype=np.float64) * 500.0
        kf.x = np.zeros((4, 1), dtype=np.float64)
        return kf

    def set_dt(self, dt_s: float) -> None:
        """Örnekleme periyodunu günceller (F ve Q yeniden kurulur)."""
        if dt_s <= 0.0:
            raise ValueError("dt_s pozitif olmalı")
        self.dt_s = float(dt_s)
        x = self._kf.x.copy()
        p = self._kf.P.copy()
        initialized = self._initialized
        count = self._update_count
        innov = self._last_innovation_norm
        self._kf = self._build_filter()
        self._kf.x = x
        self._kf.P = p
        self._initialized = initialized
        self._update_count = count
        self._last_innovation_norm = innov

    def wgs84_to_ned_m(self, point: GeoPointWGS84) -> Tuple[float, float]:
        m_lat = meters_per_deg_lat(self.origin_wgs84.latitude_deg)
        m_lon = meters_per_deg_lon(self.origin_wgs84.latitude_deg)
        north = (point.latitude_deg - self.origin_wgs84.latitude_deg) * m_lat
        east = (point.longitude_deg - self.origin_wgs84.longitude_deg) * m_lon
        return float(north), float(east)

    def ned_to_wgs84(self, north_m: float, east_m: float, alt_m: float) -> GeoPointWGS84:
        p = ned_offset_to_wgs84(self.origin_wgs84, north_m, east_m, down_m=0.0)
        return GeoPointWGS84(
            latitude_deg=p.latitude_deg,
            longitude_deg=p.longitude_deg,
            altitude_m=alt_m,
        )

    def predict(self) -> None:
        if self._initialized:
            self._kf.predict()

    def _mahalanobis2(self, z: np.ndarray) -> float:
        y = z.reshape(2, 1) - (self._kf.H @ self._kf.x)
        s = self._kf.H @ self._kf.P @ self._kf.H.T + self._kf.R
        try:
            s_inv = np.linalg.inv(s)
        except np.linalg.LinAlgError as exc:
            raise RuntimeError("inovasyon kovaryansı tekil") from exc
        return float((y.T @ s_inv @ y).item())

    def update_ned(
        self,
        north_m: float,
        east_m: float,
        alt_m: float,
        dt_s: Optional[float] = None,
        apply_gate: bool = True,
    ) -> TrackedTarget2D:
        """NED ölçümü ile predict+update; gated ölçümlerde sadece predict."""
        if not np.isfinite(north_m) or not np.isfinite(east_m):
            raise ValueError("north_m / east_m sonlu olmalı")
        if dt_s is not None and abs(dt_s - self.dt_s) > 1e-9:
            self.set_dt(dt_s)

        z = np.array([north_m, east_m], dtype=np.float64)

        if not self._initialized:
            self._kf.x = np.array(
                [[north_m], [east_m], [0.0], [0.0]], dtype=np.float64
            )
            self._kf.P = np.diag([self.measurement_var, self.measurement_var, 50.0, 50.0])
            self._initialized = True
            self._update_count = 1
            self._last_innovation_norm = 0.0
            return self._state_to_track(alt_m)

        self._kf.predict()
        innov2 = self._mahalanobis2(z)
        self._last_innovation_norm = float(np.sqrt(max(innov2, 0.0)))

        if apply_gate and innov2 > self.gate_chi2:
            # Aykırı sıçrama: durum propagate edilir, ölçüm yutulmaz
            return self._state_to_track(alt_m)

        self._kf.update(z)
        self._update_count += 1
        return self._state_to_track(alt_m)

    def update_wgs84(
        self,
        measurement: GeoPointWGS84,
        dt_s: Optional[float] = None,
        apply_gate: bool = True,
    ) -> TrackedTarget2D:
        """WGS84 ölçümünü NED'e çevirip filtreyi günceller."""
        n, e = self.wgs84_to_ned_m(measurement)
        return self.update_ned(
            north_m=n,
            east_m=e,
            alt_m=measurement.altitude_m,
            dt_s=dt_s,
            apply_gate=apply_gate,
        )

    def _state_to_track(self, alt_m: float) -> TrackedTarget2D:
        x = self._kf.x.reshape(4)
        p = self._kf.P
        cov2 = (
            float(p[0, 0]),
            float(p[0, 1]),
            float(p[1, 0]),
            float(p[1, 1]),
        )
        wgs = self.ned_to_wgs84(float(x[0]), float(x[1]), alt_m)
        return TrackedTarget2D(
            north_m=float(x[0]),
            east_m=float(x[1]),
            vel_north_mps=float(x[2]),
            vel_east_mps=float(x[3]),
            wgs84=wgs,
            innovation_norm=self._last_innovation_norm,
            covariance_ne=cov2,
            filter_converged=self._update_count >= self.converge_updates,
            update_count=self._update_count,
        )

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    @property
    def update_count(self) -> int:
        return self._update_count

    def position_covariance_3x3(self, down_var: float = 9.0) -> Tuple[float, ...]:
        """EstimatorPacket.position_covariance (N,E,D) 3x3 satır-major."""
        p = self._kf.P
        return (
            float(p[0, 0]),
            float(p[0, 1]),
            0.0,
            float(p[1, 0]),
            float(p[1, 1]),
            0.0,
            0.0,
            0.0,
            float(down_var),
        )

    def reset(self, origin_wgs84: Optional[GeoPointWGS84] = None) -> None:
        if origin_wgs84 is not None:
            self.origin_wgs84 = origin_wgs84
        self._kf = self._build_filter()
        self._initialized = False
        self._update_count = 0
        self._last_innovation_norm = 0.0
