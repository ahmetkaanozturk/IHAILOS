"""
IHAILOS — Modüller arası veri sözleşmeleri (Data Contracts).

Veri akışı:
  TelemetryPacket  : ArduPlane/MAVLink  → estimator, mission_coordinator, vision_engine
  VisionPacket     : vision_engine      → estimator
  EstimatorPacket  : estimator          → mission_coordinator
  MissionPacket    : mission_coordinator → comms_gateway + MAVLink komutları
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple


# ---------------------------------------------------------------------------
# Ortak yardımcılar
# ---------------------------------------------------------------------------


class FlightMode(str, Enum):
    MANUAL = "MANUAL"
    STABILIZE = "STABILIZE"
    FBWA = "FBWA"
    FBWB = "FBWB"
    CRUISE = "CRUISE"
    AUTO = "AUTO"
    GUIDED = "GUIDED"
    RTL = "RTL"
    LOITER = "LOITER"
    UNKNOWN = "UNKNOWN"


class MissionState(str, Enum):
    """mission_coordinator FSM durumları (architecture.md)."""

    SEARCH = "SEARCH"
    TRACK_LOCK = "TRACK_LOCK"
    APPROACH_PLAN = "APPROACH_PLAN"
    FINAL_RUN = "FINAL_RUN"
    RELEASE = "RELEASE"
    VERIFY = "VERIFY"
    RTL = "RTL"
    GO_AROUND = "GO_AROUND"


class TargetClass(str, Enum):
    UNKNOWN = "UNKNOWN"
    FRIENDLY = "FRIENDLY"
    HOSTILE = "HOSTILE"
    HSS = "HSS"  # Hava Savunma Sistemi (hakem sunucusu hedef tipi)
    VEHICLE = "VEHICLE"
    BUILDING = "BUILDING"


def _require_finite(name: str, value: float) -> None:
    if value != value:  # NaN
        raise ValueError(f"{name} NaN olamaz")
    if value in (float("inf"), float("-inf")):
        raise ValueError(f"{name} sonsuz olamaz")


# ---------------------------------------------------------------------------
# Alt yapılar
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class GeoPointWGS84:
    """WGS84 coğrafi nokta (estimator / mission)."""

    latitude_deg: float
    longitude_deg: float
    altitude_m: float = 0.0  # AMSL

    def __post_init__(self) -> None:
        _require_finite("latitude_deg", self.latitude_deg)
        _require_finite("longitude_deg", self.longitude_deg)
        _require_finite("altitude_m", self.altitude_m)
        if not (-90.0 <= self.latitude_deg <= 90.0):
            raise ValueError("latitude_deg [-90, 90] aralığında olmalı")
        if not (-180.0 <= self.longitude_deg <= 180.0):
            raise ValueError("longitude_deg [-180, 180] aralığında olmalı")


@dataclass(frozen=True)
class NEDVector:
    """North-East-Down yerel çerçeve (m veya m/s)."""

    north: float
    east: float
    down: float

    def __post_init__(self) -> None:
        _require_finite("north", self.north)
        _require_finite("east", self.east)
        _require_finite("down", self.down)


@dataclass(frozen=True)
class AttitudeRad:
    """Euler açıları (radyan): roll, pitch, yaw (NED heading)."""

    roll: float
    pitch: float
    yaw: float

    def __post_init__(self) -> None:
        _require_finite("roll", self.roll)
        _require_finite("pitch", self.pitch)
        _require_finite("yaw", self.yaw)


@dataclass(frozen=True)
class BoundingBoxXYXY:
    """Piksel uzayında eksen hizalı kutu (x1,y1,x2,y2)."""

    x1: float
    y1: float
    x2: float
    y2: float

    def __post_init__(self) -> None:
        for name, value in (
            ("x1", self.x1),
            ("y1", self.y1),
            ("x2", self.x2),
            ("y2", self.y2),
        ):
            _require_finite(name, value)
        if self.x2 < self.x1 or self.y2 < self.y1:
            raise ValueError("BoundingBox: x2>=x1 ve y2>=y1 olmalı")

    @property
    def center(self) -> Tuple[float, float]:
        return ((self.x1 + self.x2) * 0.5, (self.y1 + self.y2) * 0.5)

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1


@dataclass(frozen=True)
class Detection2D:
    """Tek bir YOLO/TensorRT tespiti."""

    detection_id: int
    class_id: int
    class_name: TargetClass
    confidence: float
    bbox: BoundingBoxXYXY
    # Opsiyonel: hedef düzlem varsayımı için piksel → ışın
    track_id: Optional[int] = None

    def __post_init__(self) -> None:
        _require_finite("confidence", self.confidence)
        if not (0.0 <= self.confidence <= 1.0):
            raise ValueError("confidence [0, 1] aralığında olmalı")


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole kamera iç parametreleri (geo-referencing)."""

    fx: float
    fy: float
    cx: float
    cy: float
    width_px: int
    height_px: int
    # Distorsiyon (OpenCV model): k1,k2,p1,p2,k3
    dist_coeffs: Tuple[float, float, float, float, float] = (0.0, 0.0, 0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        for name, value in (
            ("fx", self.fx),
            ("fy", self.fy),
            ("cx", self.cx),
            ("cy", self.cy),
        ):
            _require_finite(name, value)
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("fx ve fy pozitif olmalı")
        if self.width_px <= 0 or self.height_px <= 0:
            raise ValueError("görüntü boyutları pozitif olmalı")


@dataclass(frozen=True)
class WindEstimate:
    """Rüzgar kestirimi (yer hızı − hava hızı / Kalman)."""

    speed_mps: float
    direction_deg: float  # meteorolojik: rüzgarın geldiği yön, 0=N
    north_mps: float
    east_mps: float
    variance: float = 0.0

    def __post_init__(self) -> None:
        for name, value in (
            ("speed_mps", self.speed_mps),
            ("direction_deg", self.direction_deg),
            ("north_mps", self.north_mps),
            ("east_mps", self.east_mps),
            ("variance", self.variance),
        ):
            _require_finite(name, value)
        if self.speed_mps < 0.0:
            raise ValueError("speed_mps negatif olamaz")
        if self.variance < 0.0:
            raise ValueError("variance negatif olamaz")


@dataclass(frozen=True)
class BallisticSolution:
    """Rüzgar + balistik düzeltmeli darbe / salım noktası."""

    impact_point: GeoPointWGS84
    release_point: GeoPointWGS84
    time_of_flight_s: float
    horizontal_miss_m: float
    valid: bool

    def __post_init__(self) -> None:
        _require_finite("time_of_flight_s", self.time_of_flight_s)
        _require_finite("horizontal_miss_m", self.horizontal_miss_m)
        if self.time_of_flight_s < 0.0:
            raise ValueError("time_of_flight_s negatif olamaz")


@dataclass(frozen=True)
class SafetyGate:
    """Her kontrol döngüsünde zorunlu güvenlik kapısı."""

    flight_mode_ok: bool  # AUTO veya GUIDED
    attitude_ok: bool  # |roll|<5°, |pitch|<5°
    geofence_ok: bool
    rc_override_active: bool
    roll_deg: float
    pitch_deg: float
    telemetry_fresh: bool = False
    altitude_ok: bool = False

    def __post_init__(self) -> None:
        _require_finite("roll_deg", self.roll_deg)
        _require_finite("pitch_deg", self.pitch_deg)

    @property
    def all_clear(self) -> bool:
        return (
            self.flight_mode_ok
            and self.attitude_ok
            and self.geofence_ok
            and not self.rc_override_active
            and self.telemetry_fresh
            and self.altitude_ok
        )


@dataclass(frozen=True)
class MavlinkSetpoint:
    """GUIDED mod için konum / hız set-point."""

    target: GeoPointWGS84
    groundspeed_mps: Optional[float] = None
    yaw_deg: Optional[float] = None
    accept_radius_m: float = 15.0

    def __post_init__(self) -> None:
        _require_finite("accept_radius_m", self.accept_radius_m)
        if self.accept_radius_m <= 0.0:
            raise ValueError("accept_radius_m pozitif olmalı")
        if self.groundspeed_mps is not None:
            _require_finite("groundspeed_mps", self.groundspeed_mps)
        if self.yaw_deg is not None:
            _require_finite("yaw_deg", self.yaw_deg)


# ---------------------------------------------------------------------------
# Ana paketler
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TelemetryPacket:
    """
    ArduPlane / MAVLink telemetrisi.
    Üretici: comms_gateway (MAVLink ingest)
    Tüketiciler: vision_engine, estimator, mission_coordinator
    """

    timestamp_ns: int
    sequence: int
    position: GeoPointWGS84
    relative_alt_m: float
    attitude: AttitudeRad
    velocity_ned_mps: NEDVector
    airspeed_mps: float
    groundspeed_mps: float
    heading_deg: float
    flight_mode: FlightMode
    armed: bool
    gps_fix_num: int
    satellites_visible: int
    battery_voltage_v: float
    battery_remaining_pct: float
    home: Optional[GeoPointWGS84] = None
    throttle_pct: float = 0.0
    climb_rate_mps: float = 0.0
    # Vertical AGL derived from a downward-facing rangefinder sample.
    agl_altitude_m: Optional[float] = None
    agl_timestamp_ns: Optional[int] = None
    # Receipt times for the samples required by the mission safety gate.
    position_timestamp_ns: Optional[int] = None
    attitude_timestamp_ns: Optional[int] = None
    heartbeat_timestamp_ns: Optional[int] = None

    def __post_init__(self) -> None:
        if self.timestamp_ns < 0:
            raise ValueError("timestamp_ns negatif olamaz")
        if self.sequence < 0:
            raise ValueError("sequence negatif olamaz")
        for name, value in (
            ("position_timestamp_ns", self.position_timestamp_ns),
            ("attitude_timestamp_ns", self.attitude_timestamp_ns),
            ("heartbeat_timestamp_ns", self.heartbeat_timestamp_ns),
        ):
            if value is not None and value < 0:
                raise ValueError(f"{name} negatif olamaz")
        if (self.agl_altitude_m is None) != (self.agl_timestamp_ns is None):
            raise ValueError("AGL ölçümü ve zaman damgası birlikte verilmelidir")
        if self.agl_altitude_m is not None:
            _require_finite("agl_altitude_m", self.agl_altitude_m)
            if self.agl_altitude_m < 0.0:
                raise ValueError("agl_altitude_m negatif olamaz")
            if self.agl_timestamp_ns is None or self.agl_timestamp_ns < 0:
                raise ValueError("agl_timestamp_ns negatif olamaz")
        for name, value in (
            ("relative_alt_m", self.relative_alt_m),
            ("airspeed_mps", self.airspeed_mps),
            ("groundspeed_mps", self.groundspeed_mps),
            ("heading_deg", self.heading_deg),
            ("battery_voltage_v", self.battery_voltage_v),
            ("battery_remaining_pct", self.battery_remaining_pct),
            ("throttle_pct", self.throttle_pct),
            ("climb_rate_mps", self.climb_rate_mps),
        ):
            _require_finite(name, value)
        if not (0 <= self.gps_fix_num <= 6):
            raise ValueError("gps_fix_num [0, 6] olmalı")
        if self.satellites_visible < 0:
            raise ValueError("satellites_visible negatif olamaz")
        if not (0.0 <= self.battery_remaining_pct <= 100.0):
            raise ValueError("battery_remaining_pct [0, 100] olmalı")

    @property
    def roll_deg(self) -> float:
        return self.attitude.roll * (180.0 / 3.141592653589793)

    @property
    def pitch_deg(self) -> float:
        return self.attitude.pitch * (180.0 / 3.141592653589793)

    def attitude_within_limits(self, limit_deg: float = 5.0) -> bool:
        return abs(self.roll_deg) < limit_deg and abs(self.pitch_deg) < limit_deg

    def is_auto_or_guided(self) -> bool:
        return self.flight_mode in (FlightMode.AUTO, FlightMode.GUIDED)

    def to_dict(self) -> Dict[str, Any]:
        return _packet_to_dict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> TelemetryPacket:
        return _telemetry_from_dict(data)


@dataclass(frozen=True)
class VisionPacket:
    """
    YOLO + TensorRT tespit çıktısı (zaman damgalı).
    Üretici: vision_engine
    Tüketici: estimator
    """

    timestamp_ns: int
    frame_id: int
    sync_telemetry_seq: int  # eşzamanlı TelemetryPacket.sequence
    image_width: int
    image_height: int
    detections: Tuple[Detection2D, ...]
    camera: CameraIntrinsics
    inference_latency_ms: float
    model_name: str
    tensorrt_engine: bool = True

    def __post_init__(self) -> None:
        if self.timestamp_ns < 0:
            raise ValueError("timestamp_ns negatif olamaz")
        if self.frame_id < 0:
            raise ValueError("frame_id negatif olamaz")
        if self.sync_telemetry_seq < 0:
            raise ValueError("sync_telemetry_seq negatif olamaz")
        if self.image_width <= 0 or self.image_height <= 0:
            raise ValueError("görüntü boyutları pozitif olmalı")
        _require_finite("inference_latency_ms", self.inference_latency_ms)
        if self.inference_latency_ms < 0.0:
            raise ValueError("inference_latency_ms negatif olamaz")
        if not self.model_name:
            raise ValueError("model_name boş olamaz")

    @property
    def detection_count(self) -> int:
        return len(self.detections)

    def best_detection(
        self, class_filter: Optional[TargetClass] = None
    ) -> Optional[Detection2D]:
        pool = self.detections
        if class_filter is not None:
            pool = tuple(d for d in pool if d.class_name == class_filter)
        if not pool:
            return None
        return max(pool, key=lambda d: d.confidence)

    def to_dict(self) -> Dict[str, Any]:
        return _packet_to_dict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> VisionPacket:
        return _vision_from_dict(data)


@dataclass(frozen=True)
class EstimatorPacket:
    """
    Geo-referencing (WGS84) + Kalman + rüzgar/balistik çıktısı.
    Üretici: estimator
    Tüketici: mission_coordinator
    """

    timestamp_ns: int
    source_vision_frame_id: int
    source_telemetry_seq: int
    target_class: TargetClass
    target_wgs84: GeoPointWGS84
    target_ned_from_uav: NEDVector
    # 3x3 konum kovaryansı satır-major (m^2): N,E,D
    position_covariance: Tuple[float, ...]
    kalman_innovation_norm: float
    wind: WindEstimate
    ballistic: Optional[BallisticSolution]
    confidence: float
    filter_converged: bool
    valid: bool

    def __post_init__(self) -> None:
        if self.timestamp_ns < 0:
            raise ValueError("timestamp_ns negatif olamaz")
        if self.source_vision_frame_id < 0:
            raise ValueError("source_vision_frame_id negatif olamaz")
        if self.source_telemetry_seq < 0:
            raise ValueError("source_telemetry_seq negatif olamaz")
        if len(self.position_covariance) != 9:
            raise ValueError("position_covariance 9 elemanlı (3x3) olmalı")
        for i, value in enumerate(self.position_covariance):
            _require_finite(f"position_covariance[{i}]", value)
        _require_finite("kalman_innovation_norm", self.kalman_innovation_norm)
        _require_finite("confidence", self.confidence)
        if not (0.0 <= self.confidence <= 1.0):
            raise ValueError("confidence [0, 1] aralığında olmalı")
        if self.kalman_innovation_norm < 0.0:
            raise ValueError("kalman_innovation_norm negatif olamaz")

    def to_dict(self) -> Dict[str, Any]:
        return _packet_to_dict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> EstimatorPacket:
        return _estimator_from_dict(data)


@dataclass(frozen=True)
class MissionPacket:
    """
    FSM kararı + geofence + MAVLink komut niyeti.
    Üretici: mission_coordinator
    Tüketiciler: comms_gateway (hakem + kuyruk), MAVLink çıkışı
    """

    timestamp_ns: int
    sequence: int
    state: MissionState
    previous_state: MissionState
    safety: SafetyGate
    active_target: Optional[GeoPointWGS84]
    setpoint: Optional[MavlinkSetpoint]
    # Hakem sunucusuna iletilecek olaylar (HSS konumu vb.)
    referee_events: Tuple[Dict[str, Any], ...] = field(default_factory=tuple)
    # Yerel kuyruk / yeniden deneme için kalıcı kimlik
    command_id: str = ""
    engage_authorized: bool = False
    message: str = ""

    def __post_init__(self) -> None:
        if self.timestamp_ns < 0:
            raise ValueError("timestamp_ns negatif olamaz")
        if self.sequence < 0:
            raise ValueError("sequence negatif olamaz")
        if self.engage_authorized and not self.safety.all_clear:
            raise ValueError(
                "engage_authorized yalnızca safety.all_clear True iken verilebilir"
            )

    def to_dict(self) -> Dict[str, Any]:
        return _packet_to_dict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> MissionPacket:
        return _mission_from_dict(data)


# ---------------------------------------------------------------------------
# Serileştirme (comms_gateway yerel kuyruk tamponu)
# ---------------------------------------------------------------------------


def _packet_to_dict(obj: Any) -> Dict[str, Any]:
    raw = asdict(obj)
    return _normalize_enums(raw)


def _normalize_enums(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {k: _normalize_enums(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_normalize_enums(v) for v in value]
    return value


def _geo(data: Dict[str, Any]) -> GeoPointWGS84:
    return GeoPointWGS84(
        latitude_deg=float(data["latitude_deg"]),
        longitude_deg=float(data["longitude_deg"]),
        altitude_m=float(data.get("altitude_m", 0.0)),
    )


def _ned(data: Dict[str, Any]) -> NEDVector:
    return NEDVector(
        north=float(data["north"]),
        east=float(data["east"]),
        down=float(data["down"]),
    )


def _attitude(data: Dict[str, Any]) -> AttitudeRad:
    return AttitudeRad(
        roll=float(data["roll"]),
        pitch=float(data["pitch"]),
        yaw=float(data["yaw"]),
    )


def _bbox(data: Dict[str, Any]) -> BoundingBoxXYXY:
    return BoundingBoxXYXY(
        x1=float(data["x1"]),
        y1=float(data["y1"]),
        x2=float(data["x2"]),
        y2=float(data["y2"]),
    )


def _detection(data: Dict[str, Any]) -> Detection2D:
    return Detection2D(
        detection_id=int(data["detection_id"]),
        class_id=int(data["class_id"]),
        class_name=TargetClass(data["class_name"]),
        confidence=float(data["confidence"]),
        bbox=_bbox(data["bbox"]),
        track_id=data.get("track_id"),
    )


def _camera(data: Dict[str, Any]) -> CameraIntrinsics:
    dist = data.get("dist_coeffs", (0.0, 0.0, 0.0, 0.0, 0.0))
    return CameraIntrinsics(
        fx=float(data["fx"]),
        fy=float(data["fy"]),
        cx=float(data["cx"]),
        cy=float(data["cy"]),
        width_px=int(data["width_px"]),
        height_px=int(data["height_px"]),
        dist_coeffs=tuple(float(x) for x in dist),  # type: ignore[arg-type]
    )


def _wind(data: Dict[str, Any]) -> WindEstimate:
    return WindEstimate(
        speed_mps=float(data["speed_mps"]),
        direction_deg=float(data["direction_deg"]),
        north_mps=float(data["north_mps"]),
        east_mps=float(data["east_mps"]),
        variance=float(data.get("variance", 0.0)),
    )


def _ballistic(data: Optional[Dict[str, Any]]) -> Optional[BallisticSolution]:
    if data is None:
        return None
    return BallisticSolution(
        impact_point=_geo(data["impact_point"]),
        release_point=_geo(data["release_point"]),
        time_of_flight_s=float(data["time_of_flight_s"]),
        horizontal_miss_m=float(data["horizontal_miss_m"]),
        valid=bool(data["valid"]),
    )


def _safety(data: Dict[str, Any]) -> SafetyGate:
    return SafetyGate(
        flight_mode_ok=bool(data["flight_mode_ok"]),
        attitude_ok=bool(data["attitude_ok"]),
        geofence_ok=bool(data["geofence_ok"]),
        rc_override_active=bool(data["rc_override_active"]),
        roll_deg=float(data["roll_deg"]),
        pitch_deg=float(data["pitch_deg"]),
        telemetry_fresh=bool(data.get("telemetry_fresh", False)),
        altitude_ok=bool(data.get("altitude_ok", False)),
    )


def _setpoint(data: Optional[Dict[str, Any]]) -> Optional[MavlinkSetpoint]:
    if data is None:
        return None
    return MavlinkSetpoint(
        target=_geo(data["target"]),
        groundspeed_mps=(
            None
            if data.get("groundspeed_mps") is None
            else float(data["groundspeed_mps"])
        ),
        yaw_deg=None if data.get("yaw_deg") is None else float(data["yaw_deg"]),
        accept_radius_m=float(data.get("accept_radius_m", 15.0)),
    )


def _telemetry_from_dict(data: Dict[str, Any]) -> TelemetryPacket:
    home = data.get("home")
    return TelemetryPacket(
        timestamp_ns=int(data["timestamp_ns"]),
        sequence=int(data["sequence"]),
        position=_geo(data["position"]),
        relative_alt_m=float(data["relative_alt_m"]),
        attitude=_attitude(data["attitude"]),
        velocity_ned_mps=_ned(data["velocity_ned_mps"]),
        airspeed_mps=float(data["airspeed_mps"]),
        groundspeed_mps=float(data["groundspeed_mps"]),
        heading_deg=float(data["heading_deg"]),
        flight_mode=FlightMode(data["flight_mode"]),
        armed=bool(data["armed"]),
        gps_fix_num=int(data["gps_fix_num"]),
        satellites_visible=int(data["satellites_visible"]),
        battery_voltage_v=float(data["battery_voltage_v"]),
        battery_remaining_pct=float(data["battery_remaining_pct"]),
        home=None if home is None else _geo(home),
        throttle_pct=float(data.get("throttle_pct", 0.0)),
        climb_rate_mps=float(data.get("climb_rate_mps", 0.0)),
        agl_altitude_m=(
            None
            if data.get("agl_altitude_m") is None
            else float(data["agl_altitude_m"])
        ),
        agl_timestamp_ns=(
            None
            if data.get("agl_timestamp_ns") is None
            else int(data["agl_timestamp_ns"])
        ),
        position_timestamp_ns=(
            None
            if data.get("position_timestamp_ns") is None
            else int(data["position_timestamp_ns"])
        ),
        attitude_timestamp_ns=(
            None
            if data.get("attitude_timestamp_ns") is None
            else int(data["attitude_timestamp_ns"])
        ),
        heartbeat_timestamp_ns=(
            None
            if data.get("heartbeat_timestamp_ns") is None
            else int(data["heartbeat_timestamp_ns"])
        ),
    )


def _vision_from_dict(data: Dict[str, Any]) -> VisionPacket:
    detections = tuple(_detection(d) for d in data.get("detections", []))
    return VisionPacket(
        timestamp_ns=int(data["timestamp_ns"]),
        frame_id=int(data["frame_id"]),
        sync_telemetry_seq=int(data["sync_telemetry_seq"]),
        image_width=int(data["image_width"]),
        image_height=int(data["image_height"]),
        detections=detections,
        camera=_camera(data["camera"]),
        inference_latency_ms=float(data["inference_latency_ms"]),
        model_name=str(data["model_name"]),
        tensorrt_engine=bool(data.get("tensorrt_engine", True)),
    )


def _estimator_from_dict(data: Dict[str, Any]) -> EstimatorPacket:
    cov = tuple(float(x) for x in data["position_covariance"])
    return EstimatorPacket(
        timestamp_ns=int(data["timestamp_ns"]),
        source_vision_frame_id=int(data["source_vision_frame_id"]),
        source_telemetry_seq=int(data["source_telemetry_seq"]),
        target_class=TargetClass(data["target_class"]),
        target_wgs84=_geo(data["target_wgs84"]),
        target_ned_from_uav=_ned(data["target_ned_from_uav"]),
        position_covariance=cov,
        kalman_innovation_norm=float(data["kalman_innovation_norm"]),
        wind=_wind(data["wind"]),
        ballistic=_ballistic(data.get("ballistic")),
        confidence=float(data["confidence"]),
        filter_converged=bool(data["filter_converged"]),
        valid=bool(data["valid"]),
    )


def _mission_from_dict(data: Dict[str, Any]) -> MissionPacket:
    active = data.get("active_target")
    events = data.get("referee_events", [])
    return MissionPacket(
        timestamp_ns=int(data["timestamp_ns"]),
        sequence=int(data["sequence"]),
        state=MissionState(data["state"]),
        previous_state=MissionState(data["previous_state"]),
        safety=_safety(data["safety"]),
        active_target=None if active is None else _geo(active),
        setpoint=_setpoint(data.get("setpoint")),
        referee_events=tuple(dict(e) for e in events),
        command_id=str(data.get("command_id", "")),
        engage_authorized=bool(data.get("engage_authorized", False)),
        message=str(data.get("message", "")),
    )
