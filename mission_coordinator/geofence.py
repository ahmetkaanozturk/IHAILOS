"""
Geofence: Shapely ile hedef ve yaklaşma koridoru poligon denetimi.

Görev alanı (ops polygon) + FAW→Release→Target koridoru (approach corridor).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
from shapely.validation import make_valid

from common.data_contracts import GeoPointWGS84
from estimator.geo_referencing import meters_per_deg_lat, meters_per_deg_lon

LonLat = Tuple[float, float]  # (lon, lat) — Shapely coğrafi sıra


def _as_lonlat(point: GeoPointWGS84) -> LonLat:
    return (point.longitude_deg, point.latitude_deg)


def _ring_closed(vertices: Sequence[LonLat]) -> List[LonLat]:
    if len(vertices) < 3:
        raise ValueError("poligon en az 3 köşe gerektirir")
    ring = [(float(lon), float(lat)) for lon, lat in vertices]
    for lon, lat in ring:
        if not (-180.0 <= lon <= 180.0 and -90.0 <= lat <= 90.0):
            raise ValueError(f"geçersiz köşe: lon={lon}, lat={lat}")
    if ring[0] != ring[-1]:
        ring.append(ring[0])
    return ring


def build_polygon(vertices_lonlat: Sequence[LonLat]) -> Polygon:
    """(lon, lat) köşe listesinden geçerli Shapely Polygon üretir."""
    ring = _ring_closed(vertices_lonlat)
    poly = Polygon(ring)
    if not poly.is_valid:
        poly = make_valid(poly)
        if poly.geom_type == "MultiPolygon":
            poly = max(poly.geoms, key=lambda g: g.area)
        if poly.geom_type != "Polygon":
            raise ValueError(f"poligon düzeltilemedi: {poly.geom_type}")
    if poly.is_empty or poly.area <= 0.0:
        raise ValueError("poligon alanı sıfır")
    return poly


def corridor_polygon_wgs84(
    path_points: Sequence[GeoPointWGS84],
    half_width_m: float,
    reference_lat_deg: float,
) -> Polygon:
    """
    WGS84 nokta dizisi boyunca metre cinsinden yarı-genişlikli koridor poligonu.

    Derece uzayında buffer için metre→derece ölçekleri reference_lat ile alınır.
    """
    if len(path_points) < 2:
        raise ValueError("koridor için en az 2 nokta gerekir")
    if half_width_m <= 0.0:
        raise ValueError("half_width_m pozitif olmalı")

    m_lat = meters_per_deg_lat(reference_lat_deg)
    m_lon = meters_per_deg_lon(reference_lat_deg)
    if m_lat < 1e-3 or abs(m_lon) < 1e-3:
        raise ValueError("metre/derece ölçeği geçersiz")

    # Anizotropik buffer: lon/lat'ı lokal metrik düzleme projekte et
    coords_m: List[Tuple[float, float]] = []
    origin = path_points[0]
    for p in path_points:
        x = (p.longitude_deg - origin.longitude_deg) * m_lon
        y = (p.latitude_deg - origin.latitude_deg) * m_lat
        coords_m.append((x, y))

    line_m = LineString(coords_m)
    buf_m = line_m.buffer(half_width_m, cap_style=2, join_style=2)
    if buf_m.is_empty:
        raise ValueError("koridor buffer boş")

    # Metrik → lon/lat
    if buf_m.geom_type == "Polygon":
        exteriors = [buf_m.exterior.coords]
    elif buf_m.geom_type == "MultiPolygon":
        exteriors = [g.exterior.coords for g in buf_m.geoms]
    else:
        raise ValueError(f"beklenmeyen buffer tipi: {buf_m.geom_type}")

    polys: List[Polygon] = []
    for exterior in exteriors:
        lonlat = [
            (
                origin.longitude_deg + x / m_lon,
                origin.latitude_deg + y / m_lat,
            )
            for x, y in exterior
        ]
        polys.append(build_polygon(lonlat))

    merged = unary_union(polys)
    if merged.geom_type == "MultiPolygon":
        merged = max(merged.geoms, key=lambda g: g.area)
    if merged.geom_type != "Polygon":
        raise ValueError("koridor birleştirilemedi")
    return merged


@dataclass(frozen=True)
class GeofenceVerdict:
    """Tek bir denetim sonucu."""

    target_inside: bool
    approach_inside: bool
    uav_inside_ops: bool
    message: str

    @property
    def ok_for_engage(self) -> bool:
        return self.target_inside and self.approach_inside and self.uav_inside_ops


class MissionGeofence:
    """
    Operasyon poligonu + dinamik yaklaşma koridoru denetçisi.
    """

    def __init__(
        self,
        ops_vertices_lonlat: Sequence[LonLat],
        corridor_half_width_m: float = 75.0,
    ) -> None:
        self._ops = build_polygon(ops_vertices_lonlat)
        if corridor_half_width_m <= 0.0:
            raise ValueError("corridor_half_width_m pozitif olmalı")
        self._corridor_half_width_m = float(corridor_half_width_m)
        self._approach: Polygon | None = None

    @property
    def ops_polygon(self) -> Polygon:
        return self._ops

    @property
    def approach_polygon(self) -> Polygon | None:
        return self._approach

    @property
    def corridor_half_width_m(self) -> float:
        return self._corridor_half_width_m

    def contains_wgs84(self, point: GeoPointWGS84, polygon: Polygon) -> bool:
        return polygon.covers(Point(_as_lonlat(point)))

    def target_inside(self, target: GeoPointWGS84) -> bool:
        return self.contains_wgs84(target, self._ops)

    def uav_inside_ops(self, uav: GeoPointWGS84) -> bool:
        return self.contains_wgs84(uav, self._ops)

    def set_approach_corridor(
        self,
        faw: GeoPointWGS84,
        release_point: GeoPointWGS84,
        target: GeoPointWGS84,
    ) -> Polygon:
        """FAW → Release → Target koridorunu günceller."""
        ref_lat = 0.5 * (faw.latitude_deg + target.latitude_deg)
        self._approach = corridor_polygon_wgs84(
            path_points=(faw, release_point, target),
            half_width_m=self._corridor_half_width_m,
            reference_lat_deg=ref_lat,
        )
        # Koridor operasyon alanı ile kesişmeli
        if self._approach.intersection(self._ops).is_empty:
            raise ValueError("yaklaşma koridoru operasyon poligonu ile kesişmiyor")
        return self._approach

    def clear_approach_corridor(self) -> None:
        self._approach = None

    def approach_path_inside(
        self,
        points: Iterable[GeoPointWGS84],
    ) -> bool:
        """Koridor tanımlıysa tüm noktalar koridor∩ops içinde olmalı."""
        pts = list(points)
        if not pts:
            return False
        if self._approach is None:
            return all(self.target_inside(p) for p in pts)
        zone = self._ops.intersection(self._approach)
        if zone.is_empty:
            return False
        return all(zone.covers(Point(_as_lonlat(p))) for p in pts)

    def evaluate(
        self,
        uav: GeoPointWGS84,
        target: GeoPointWGS84 | None,
        approach_points: Sequence[GeoPointWGS84] = (),
    ) -> GeofenceVerdict:
        """Hedef + yaklaşma + İHA için birleşik geofence kararı."""
        uav_ok = self.uav_inside_ops(uav)
        if target is None:
            return GeofenceVerdict(
                target_inside=False,
                approach_inside=False,
                uav_inside_ops=uav_ok,
                message="hedef yok",
            )
        tgt_ok = self.target_inside(target)
        if approach_points:
            appr_ok = self.approach_path_inside(approach_points)
        elif self._approach is not None:
            appr_ok = self.contains_wgs84(target, self._ops.intersection(self._approach))
        else:
            appr_ok = tgt_ok

        if not uav_ok:
            msg = "İHA operasyon poligonu dışında"
        elif not tgt_ok:
            msg = "hedef operasyon poligonu dışında"
        elif not appr_ok:
            msg = "yaklaşma koridoru ihlali"
        else:
            msg = "geofence OK"

        return GeofenceVerdict(
            target_inside=tgt_ok,
            approach_inside=appr_ok,
            uav_inside_ops=uav_ok,
            message=msg,
        )
