"""
Geo-referencing: piksel (u, v) → WGS84 (lat, lon).

Bölüm 2 — Estimator / Geo-referencing:
  1) Pinhole:  ray_c = K^{-1} [u, v, 1]^T
  2) Kamera→gövde: OpenCV kamera çerçevesi + Pitch_mount = 30° öne eğim
  3) Gövde→NED: R = Rz(yaw) Ry(pitch) Rx(roll)
  4) LiDAR AGL (h) ile yer düzlemi kesişimi
  5) Yerel NED ofset → WGS84
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np

from common.data_contracts import AttitudeRad, CameraIntrinsics, GeoPointWGS84

# WGS84 yarı-eksenleri (m)
_WGS84_A = 6378137.0
_WGS84_F = 1.0 / 298.257223563
_WGS84_E2 = _WGS84_F * (2.0 - _WGS84_F)

# Mimari varsayılan: kamera gövde X eksenine göre 30° öne (aşağı) eğik
DEFAULT_PITCH_MOUNT_DEG = 30.0


def rotation_matrix_x(angle_rad: float) -> np.ndarray:
    c = float(np.cos(angle_rad))
    s = float(np.sin(angle_rad))
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)


def rotation_matrix_y(angle_rad: float) -> np.ndarray:
    c = float(np.cos(angle_rad))
    s = float(np.sin(angle_rad))
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def rotation_matrix_z(angle_rad: float) -> np.ndarray:
    c = float(np.cos(angle_rad))
    s = float(np.sin(angle_rad))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def camera_intrinsics_matrix(camera: CameraIntrinsics) -> np.ndarray:
    """Pinhole iç matris K (3x3)."""
    return np.array(
        [
            [camera.fx, 0.0, camera.cx],
            [0.0, camera.fy, camera.cy],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def opencv_camera_to_body_frd(pitch_mount_deg: float = DEFAULT_PITCH_MOUNT_DEG) -> np.ndarray:
    """
    OpenCV kamera → gövde FRD (X ileri, Y sağ, Z aşağı).

    OpenCV: X sağ, Y aşağı, Z optik eksen (ileri).
    R_cv_frd: cam_z→body_x, cam_x→body_y, cam_y→body_z.
    Pitch_mount (öne eğim): optik ekseni gövde X'ten Pitch_mount kadar +Z (aşağı)
    eğer → FRD'de Ry(-Pitch_mount).
    """
    r_cv_to_aligned = np.array(
        [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=np.float64,
    )
    pitch_mount_rad = np.deg2rad(pitch_mount_deg)
    r_mount = rotation_matrix_y(-pitch_mount_rad)
    return r_mount @ r_cv_to_aligned


def body_to_ned_rotation(attitude: AttitudeRad) -> np.ndarray:
    """
    Gövde FRD → NED: R_ned_body = Rz(yaw) @ Ry(pitch) @ Rx(roll).
    Sütunlar: gövde baz vektörlerinin NED bileşenleri.
    """
    return (
        rotation_matrix_z(attitude.yaw)
        @ rotation_matrix_y(attitude.pitch)
        @ rotation_matrix_x(attitude.roll)
    )


def pixel_to_camera_ray(u: float, v: float, k_matrix: np.ndarray) -> np.ndarray:
    """Normalize edilmemiş kamera ışını: K^{-1} [u,v,1]^T."""
    if k_matrix.shape != (3, 3):
        raise ValueError("K 3x3 olmalı")
    uv1 = np.array([u, v, 1.0], dtype=np.float64)
    ray = np.linalg.inv(k_matrix) @ uv1
    norm = float(np.linalg.norm(ray))
    if norm < 1e-12:
        raise ValueError("kamera ışını sıfır vektör")
    return ray / norm


def intersect_ray_with_ground_ned(
    direction_ned: np.ndarray,
    agl_m: float,
) -> Tuple[float, float, float]:
    """
    İHA NED orijinde (0,0,0), yer düzlemi Down = agl_m.
    P(t) = t * d_ned, t = agl / d_down  (d_down > 0 şart).
    Dönüş: (north_m, east_m, down_m).
    """
    if agl_m <= 0.0:
        raise ValueError("LiDAR AGL pozitif olmalı")
    d = np.asarray(direction_ned, dtype=np.float64).reshape(3)
    d_norm = float(np.linalg.norm(d))
    if d_norm < 1e-12:
        raise ValueError("NED ışın yönü sıfır")
    d = d / d_norm
    d_down = float(d[2])
    if d_down <= 1e-6:
        raise ValueError(
            "Işın yere değmiyor (d_down<=0); tutum/kamera açısı veya piksel geçersiz"
        )
    t = agl_m / d_down
    point = t * d
    return float(point[0]), float(point[1]), float(point[2])


def meters_per_deg_lat(latitude_deg: float) -> float:
    """WGS84 meridyen yay uzunluğu (m/deg) — yerel teğet düzlem."""
    lat = np.deg2rad(latitude_deg)
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    denom = (1.0 - _WGS84_E2 * sin_lat * sin_lat) ** 1.5
    m_per_rad = _WGS84_A * (1.0 - _WGS84_E2) / denom
    return float(m_per_rad * np.pi / 180.0)


def meters_per_deg_lon(latitude_deg: float) -> float:
    """WGS84 paralel yay uzunluğu (m/deg)."""
    lat = np.deg2rad(latitude_deg)
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    denom = np.sqrt(1.0 - _WGS84_E2 * sin_lat * sin_lat)
    n_per_rad = _WGS84_A / denom
    return float(n_per_rad * cos_lat * np.pi / 180.0)


def ned_offset_to_wgs84(
    origin: GeoPointWGS84,
    north_m: float,
    east_m: float,
    down_m: float = 0.0,
) -> GeoPointWGS84:
    """Yerel NED ofsetini WGS84 lat/lon/alt'a çevirir (düzlem yaklaşımı)."""
    m_lat = meters_per_deg_lat(origin.latitude_deg)
    m_lon = meters_per_deg_lon(origin.latitude_deg)
    if m_lat < 1e-6 or abs(m_lon) < 1e-6:
        raise ValueError("geçersiz enlem için m/deg hesabı")
    lat = origin.latitude_deg + north_m / m_lat
    lon = origin.longitude_deg + east_m / m_lon
    alt = origin.altitude_m - down_m
    if lon > 180.0:
        lon -= 360.0
    elif lon < -180.0:
        lon += 360.0
    lat = float(np.clip(lat, -90.0, 90.0))
    return GeoPointWGS84(latitude_deg=lat, longitude_deg=lon, altitude_m=alt)


@dataclass(frozen=True)
class GeoReferenceResult:
    """Piksel→WGS84 ara çıktıları (NED ofset + coğrafi nokta)."""

    target_wgs84: GeoPointWGS84
    north_m: float
    east_m: float
    down_m: float
    ray_ned: Tuple[float, float, float]
    range_m: float


def pixel_to_wgs84(
    u: float,
    v: float,
    camera: CameraIntrinsics,
    attitude: AttitudeRad,
    uav_position: GeoPointWGS84,
    lidar_agl_m: float,
    pitch_mount_deg: float = DEFAULT_PITCH_MOUNT_DEG,
) -> GeoReferenceResult:
    """
    (u, v) piksel merkezini LiDAR AGL ve tutum ile WGS84'e projekte eder.

    Parametreler
    ----------
    u, v : piksel koordinatları (OpenCV: sol-üst orijin)
    camera : fx, fy, cx, cy içeren iç parametreler
    attitude : İHA roll/pitch/yaw (radyan, NED)
    uav_position : GPS WGS84 (AMSL)
    lidar_agl_m : LiDAR yer irtifası (m)
    pitch_mount_deg : kamera montaj eğimi (varsayılan 30°)
    """
    if not np.isfinite(u) or not np.isfinite(v):
        raise ValueError("u, v sonlu olmalı")
    if lidar_agl_m <= 0.0:
        raise ValueError("lidar_agl_m pozitif olmalı")

    k = camera_intrinsics_matrix(camera)
    ray_c = pixel_to_camera_ray(u, v, k)
    r_c_b = opencv_camera_to_body_frd(pitch_mount_deg)
    ray_body = r_c_b @ ray_c
    r_b_n = body_to_ned_rotation(attitude)
    ray_ned = r_b_n @ ray_body

    north_m, east_m, down_m = intersect_ray_with_ground_ned(ray_ned, lidar_agl_m)
    range_m = float(np.sqrt(north_m * north_m + east_m * east_m + down_m * down_m))
    target = ned_offset_to_wgs84(uav_position, north_m, east_m, down_m)

    return GeoReferenceResult(
        target_wgs84=target,
        north_m=north_m,
        east_m=east_m,
        down_m=down_m,
        ray_ned=(float(ray_ned[0]), float(ray_ned[1]), float(ray_ned[2])),
        range_m=range_m,
    )


def bbox_center_to_wgs84(
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    camera: CameraIntrinsics,
    attitude: AttitudeRad,
    uav_position: GeoPointWGS84,
    lidar_agl_m: float,
    pitch_mount_deg: float = DEFAULT_PITCH_MOUNT_DEG,
) -> GeoReferenceResult:
    """YOLO bbox (xyxy) merkezinden WGS84 üretir."""
    u = 0.5 * (x1 + x2)
    v = 0.5 * (y1 + y2)
    return pixel_to_wgs84(
        u=u,
        v=v,
        camera=camera,
        attitude=attitude,
        uav_position=uav_position,
        lidar_agl_m=lidar_agl_m,
        pitch_mount_deg=pitch_mount_deg,
    )
