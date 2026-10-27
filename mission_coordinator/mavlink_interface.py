"""
MAVLink arayüzü: zaman damgalı ring-buffer telemetri, RC override,
GUIDED waypoint, AUX9 DO_SET_SERVO (PWM 2000).
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Optional, Tuple

from pymavlink import mavutil

from common.data_contracts import (
    AttitudeRad,
    FlightMode,
    GeoPointWGS84,
    MavlinkSetpoint,
    NEDVector,
    SafetyGate,
    TelemetryPacket,
)

# ArduPlane HEARTBEAT.custom_mode (ardupilotmega)
_PLANE_CUSTOM_MODE: Dict[int, FlightMode] = {
    0: FlightMode.MANUAL,
    2: FlightMode.STABILIZE,
    5: FlightMode.FBWA,
    6: FlightMode.FBWB,
    7: FlightMode.CRUISE,
    10: FlightMode.AUTO,
    11: FlightMode.RTL,
    12: FlightMode.LOITER,
    15: FlightMode.GUIDED,
}

_ALLOWED_AUTO_MODES = frozenset({FlightMode.AUTO, FlightMode.GUIDED})
ATTITUDE_LIMIT_DEG = 5.0
DEFAULT_SERVO_INSTANCE_AUX9 = 9
RELEASE_PWM_US = 2000
RING_BUFFER_DEFAULT = 256


@dataclass(frozen=True)
class TimestampedTelemetry:
    """Ring-buffer kaydı."""

    recv_timestamp_ns: int
    packet: TelemetryPacket


class MavlinkInterface:
    """
    pymavlink tabanlı uçuş kartı köprüsü.

    - Telemetriyi zaman damgalı ring-buffer'da tutar
    - Her döngüde RC Override + |Roll|/|Pitch| < 5° güvenlik kapısı
    - GUIDED global waypoint gönderir
    - AUX 9 / SERVO9 için DO_SET_SERVO (PWM 2000) tetikler
    """

    def __init__(
        self,
        connection_string: str = "udp:127.0.0.1:14550",
        ring_size: int = RING_BUFFER_DEFAULT,
        attitude_limit_deg: float = ATTITUDE_LIMIT_DEG,
        servo_instance_aux9: int = DEFAULT_SERVO_INSTANCE_AUX9,
        release_pwm_us: int = RELEASE_PWM_US,
        master: Optional[object] = None,
    ) -> None:
        if ring_size < 1:
            raise ValueError("ring_size >= 1 olmalı")
        if attitude_limit_deg <= 0.0:
            raise ValueError("attitude_limit_deg pozitif olmalı")
        if servo_instance_aux9 < 1:
            raise ValueError("servo_instance_aux9 >= 1 olmalı")
        if not (1000 <= release_pwm_us <= 2000):
            raise ValueError("release_pwm_us [1000, 2000] olmalı")

        self.connection_string = connection_string
        self.attitude_limit_deg = float(attitude_limit_deg)
        self.servo_instance_aux9 = int(servo_instance_aux9)
        self.release_pwm_us = int(release_pwm_us)

        self._ring: Deque[TimestampedTelemetry] = deque(maxlen=int(ring_size))
        self._sequence = 0
        self._master = master
        self._target_system = 1
        self._target_component = 1

        # HAM MAVLink biriktiricileri (mesaj birleşimi)
        self._lat_e7 = 0
        self._lon_e7 = 0
        self._alt_mm = 0
        self._rel_alt_mm = 0
        self._vx_cms = 0
        self._vy_cms = 0
        self._vz_cms = 0
        self._hdg_cdeg = 0
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._airspeed = 0.0
        self._groundspeed = 0.0
        self._throttle = 0.0
        self._climb = 0.0
        self._gps_fix = 0
        self._satellites = 0
        self._batt_v = 0.0
        self._batt_pct = 100.0
        self._armed = False
        self._flight_mode = FlightMode.UNKNOWN
        self._home: Optional[GeoPointWGS84] = None
        self._have_global = False
        self._have_attitude = False
        self._position_timestamp_ns: Optional[int] = None
        self._attitude_timestamp_ns: Optional[int] = None
        self._heartbeat_timestamp_ns: Optional[int] = None
        self._agl_altitude_m: Optional[float] = None
        self._agl_timestamp_ns: Optional[int] = None

    @property
    def master(self) -> object:
        if self._master is None:
            raise RuntimeError("MAVLink bağlantısı yok; connect() çağırın")
        return self._master

    @property
    def connected(self) -> bool:
        return self._master is not None

    @property
    def ring_len(self) -> int:
        return len(self._ring)

    def connect(self, timeout_s: float = 10.0) -> None:
        """pymavlink mavutil bağlantısı + heartbeat bekleme."""
        if self._master is not None:
            return
        master = mavutil.mavlink_connection(self.connection_string, autoreconnect=True)
        deadline = time.time() + timeout_s
        hb = None
        while time.time() < deadline:
            hb = master.wait_heartbeat(timeout=1)
            if hb is not None:
                break
        if hb is None:
            master.close()
            raise TimeoutError(
                f"heartbeat alınamadı: {self.connection_string} ({timeout_s}s)"
            )
        self._master = master
        self._target_system = int(master.target_system)
        self._target_component = int(master.target_component)

    def close(self) -> None:
        if self._master is not None:
            close_fn = getattr(self._master, "close", None)
            if callable(close_fn):
                close_fn()
        self._master = None

    def _decode_flight_mode(self, msg: object) -> FlightMode:
        base_mode = int(getattr(msg, "base_mode", 0))
        custom = int(getattr(msg, "custom_mode", 0))
        self._armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        mav_type = int(getattr(msg, "type", mavutil.mavlink.MAV_TYPE_FIXED_WING))
        autopilot = int(getattr(msg, "autopilot", 0))
        if autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            return _PLANE_CUSTOM_MODE.get(custom, FlightMode.UNKNOWN)
        # Jenerik: custom mode yoksa UNKNOWN
        _ = mav_type
        return _PLANE_CUSTOM_MODE.get(custom, FlightMode.UNKNOWN)

    def _ingest_mavlink_message(self, msg: object) -> None:
        mtype = msg.get_type()
        if mtype == "HEARTBEAT":
            self._flight_mode = self._decode_flight_mode(msg)
            self._heartbeat_timestamp_ns = time.time_ns()
        elif mtype == "GLOBAL_POSITION_INT":
            self._lat_e7 = int(msg.lat)
            self._lon_e7 = int(msg.lon)
            self._alt_mm = int(msg.alt)
            self._rel_alt_mm = int(msg.relative_alt)
            self._vx_cms = int(msg.vx)
            self._vy_cms = int(msg.vy)
            self._vz_cms = int(msg.vz)
            self._hdg_cdeg = int(msg.hdg)
            self._have_global = True
            self._position_timestamp_ns = time.time_ns()
        elif mtype == "ATTITUDE":
            self._roll = float(msg.roll)
            self._pitch = float(msg.pitch)
            self._yaw = float(msg.yaw)
            self._have_attitude = True
            self._attitude_timestamp_ns = time.time_ns()
        elif mtype == "DISTANCE_SENSOR":
            orientation = int(getattr(msg, "orientation", -1))
            min_distance_m = float(getattr(msg, "min_distance", 0)) / 100.0
            max_distance_m = float(getattr(msg, "max_distance", 0)) / 100.0
            distance_m = float(getattr(msg, "current_distance", 0)) / 100.0
            if orientation == mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270:
                self._agl_altitude_m = None
                self._agl_timestamp_ns = None
                if (
                    self._have_attitude
                    and min_distance_m <= distance_m <= max_distance_m
                    and distance_m > 0.0
                ):
                    vertical_scale = math.cos(self._roll) * math.cos(self._pitch)
                    agl_m = distance_m * vertical_scale
                    if math.isfinite(agl_m) and agl_m >= 0.0:
                        self._agl_altitude_m = agl_m
                        self._agl_timestamp_ns = time.time_ns()
        elif mtype == "VFR_HUD":
            self._airspeed = float(msg.airspeed)
            self._groundspeed = float(msg.groundspeed)
            self._throttle = float(msg.throttle)
            self._climb = float(msg.climb)
            if not self._have_global:
                self._hdg_cdeg = int(float(msg.heading) * 100.0)
        elif mtype == "GPS_RAW_INT":
            self._gps_fix = int(msg.fix_num)
            self._satellites = int(msg.satellites_visible)
        elif mtype == "SYS_STATUS":
            self._batt_v = float(msg.voltage_battery) / 1000.0
            rem = int(msg.battery_remaining)
            self._batt_pct = float(rem) if rem >= 0 else self._batt_pct
        elif mtype == "HOME_POSITION":
            self._home = GeoPointWGS84(
                latitude_deg=float(msg.latitude) / 1e7,
                longitude_deg=float(msg.longitude) / 1e7,
                altitude_m=float(msg.altitude) / 1000.0,
            )

    def _build_telemetry_packet(self, timestamp_ns: int) -> TelemetryPacket:
        heading = float(self._hdg_cdeg) / 100.0
        if heading > 360.0:
            heading = 0.0
        return TelemetryPacket(
            timestamp_ns=timestamp_ns,
            sequence=self._sequence,
            position=GeoPointWGS84(
                latitude_deg=float(self._lat_e7) / 1e7,
                longitude_deg=float(self._lon_e7) / 1e7,
                altitude_m=float(self._alt_mm) / 1000.0,
            ),
            relative_alt_m=float(self._rel_alt_mm) / 1000.0,
            attitude=AttitudeRad(
                roll=self._roll, pitch=self._pitch, yaw=self._yaw
            ),
            velocity_ned_mps=NEDVector(
                north=float(self._vx_cms) / 100.0,
                east=float(self._vy_cms) / 100.0,
                down=float(self._vz_cms) / 100.0,
            ),
            airspeed_mps=self._airspeed,
            groundspeed_mps=self._groundspeed,
            heading_deg=heading,
            flight_mode=self._flight_mode,
            armed=self._armed,
            gps_fix_num=self._gps_fix,
            satellites_visible=self._satellites,
            battery_voltage_v=self._batt_v,
            battery_remaining_pct=max(0.0, min(100.0, self._batt_pct)),
            home=self._home,
            throttle_pct=self._throttle,
            climb_rate_mps=self._climb,
            agl_altitude_m=self._agl_altitude_m,
            agl_timestamp_ns=self._agl_timestamp_ns,
            position_timestamp_ns=self._position_timestamp_ns,
            attitude_timestamp_ns=self._attitude_timestamp_ns,
            heartbeat_timestamp_ns=self._heartbeat_timestamp_ns,
        )

    def push_telemetry(self, packet: TelemetryPacket, recv_timestamp_ns: Optional[int] = None) -> None:
        """Sentetik / harici TelemetryPacket'i ring-buffer'a yazar."""
        ts = recv_timestamp_ns if recv_timestamp_ns is not None else time.time_ns()
        self._sequence = packet.sequence
        self._flight_mode = packet.flight_mode
        self._armed = packet.armed
        self._roll = packet.attitude.roll
        self._pitch = packet.attitude.pitch
        self._yaw = packet.attitude.yaw
        self._ring.append(TimestampedTelemetry(recv_timestamp_ns=ts, packet=packet))

    def spin_once(self, timeout_s: float = 0.05) -> Optional[TelemetryPacket]:
        """
        Bir MAVLink mesajı oku; GLOBAL+ATTITUDE hazırsa packet üretip ring'e koy.
        """
        master = self.master
        msg = master.recv_match(blocking=False, timeout=timeout_s)
        if msg is None:
            return None
        self._ingest_mavlink_message(msg)
        if not (self._have_global and self._have_attitude):
            return None
        self._sequence += 1
        ts = time.time_ns()
        packet = self._build_telemetry_packet(ts)
        self._ring.append(TimestampedTelemetry(recv_timestamp_ns=ts, packet=packet))
        return packet

    def latest(self) -> Optional[TelemetryPacket]:
        if not self._ring:
            return None
        return self._ring[-1].packet

    def latest_timestamped(self) -> Optional[TimestampedTelemetry]:
        if not self._ring:
            return None
        return self._ring[-1]

    def history(self) -> Tuple[TimestampedTelemetry, ...]:
        return tuple(self._ring)

    def is_rc_override_active(self, packet: Optional[TelemetryPacket] = None) -> bool:
        """
        architecture.md: flight_mode not in ['AUTO', 'GUIDED'] ⇒ RC Override.
        """
        pkt = packet if packet is not None else self.latest()
        if pkt is None:
            return True
        return pkt.flight_mode not in _ALLOWED_AUTO_MODES

    def evaluate_safety(self, packet: Optional[TelemetryPacket] = None) -> SafetyGate:
        """Her kontrol döngüsünde zorunlu güvenlik kapısı."""
        pkt = packet if packet is not None else self.latest()
        if pkt is None:
            return SafetyGate(
                flight_mode_ok=False,
                attitude_ok=False,
                geofence_ok=False,
                rc_override_active=True,
                roll_deg=0.0,
                pitch_deg=0.0,
                telemetry_fresh=False,
                altitude_ok=False,
            )
        roll_deg = pkt.roll_deg
        pitch_deg = pkt.pitch_deg
        mode_ok = pkt.flight_mode in _ALLOWED_AUTO_MODES
        attitude_ok = (
            abs(roll_deg) < self.attitude_limit_deg
            and abs(pitch_deg) < self.attitude_limit_deg
        )
        rc_override = not mode_ok
        return SafetyGate(
            flight_mode_ok=mode_ok,
            attitude_ok=attitude_ok,
            geofence_ok=True,  # geofence mission_coordinator katmanında güncellenir
            rc_override_active=rc_override,
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
        )

    def set_mode_guided(self) -> None:
        master = self.master
        master.mav.command_long_send(
            self._target_system,
            self._target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            15.0,  # ArduPlane GUIDED
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def send_guided_waypoint(self, setpoint: MavlinkSetpoint) -> None:
        """
        GUIDED konum hedefi (GLOBAL_RELATIVE_ALT_INT).
        type_mask: yalnızca konum kullanılır; hız/ivme/yaw yok sayılır.
        """
        master = self.master
        lat_e7 = int(round(setpoint.target.latitude_deg * 1e7))
        lon_e7 = int(round(setpoint.target.longitude_deg * 1e7))
        # relative alt: setpoint AMSL farkı bilinmiyorsa altitude_m relative kabul
        alt_m = float(setpoint.target.altitude_m)

        # bit0-2 pos ignore=0 (kullan), bit3-5 vel ignore, bit6-8 accel ignore,
        # bit9 force force, bit10 yaw ignore, bit11 yaw_rate ignore
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        yaw = 0.0
        if setpoint.yaw_deg is not None:
            type_mask &= ~mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            yaw = math.radians(setpoint.yaw_deg)

        master.mav.set_position_target_global_int_send(
            0,  # time_boot_ms
            self._target_system,
            self._target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            lat_e7,
            lon_e7,
            alt_m,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            yaw,
            0.0,
        )

        if setpoint.groundspeed_mps is not None:
            master.mav.command_long_send(
                self._target_system,
                self._target_component,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0,
                1.0,  # groundspeed
                float(setpoint.groundspeed_mps),
                -1.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )

    def trigger_release_servo_aux9(self, pwm_us: Optional[int] = None) -> None:
        """
        MAV_CMD_DO_SET_SERVO → AUX/SERVO instance 9, PWM 2000 (salım).
        """
        master = self.master
        pwm = int(self.release_pwm_us if pwm_us is None else pwm_us)
        if not (1000 <= pwm <= 2000):
            raise ValueError("pwm [1000, 2000] olmalı")
        master.mav.command_long_send(
            self._target_system,
            self._target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            float(self.servo_instance_aux9),
            float(pwm),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def set_mode_rtl(self) -> None:
        master = self.master
        master.mav.command_long_send(
            self._target_system,
            self._target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
