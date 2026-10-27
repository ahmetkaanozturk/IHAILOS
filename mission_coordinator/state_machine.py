"""
Görev FSM: SEARCH → TRACK_LOCK → APPROACH_PLAN → FINAL_RUN → RELEASE → VERIFY → RTL
+ GO_AROUND (güvenlik / geofence ihlali).

Her tick'te architecture.md güvenlik kuralı uygulanır:
  RC Override (mode ∉ {AUTO, GUIDED}) veya |Roll|≥5° veya |Pitch|≥5° ⇒ salım yok / GO_AROUND.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

from common.data_contracts import (
    EstimatorPacket,
    GeoPointWGS84,
    MavlinkSetpoint,
    MissionPacket,
    MissionState,
    SafetyGate,
    TelemetryPacket,
)
from estimator.ballistics import BallisticPlan, FAW_STANDOFF_M, plan_ballistic_engagement
from estimator.geo_referencing import meters_per_deg_lat, meters_per_deg_lon
from mission_coordinator.geofence import MissionGeofence
from mission_coordinator.mavlink_interface import ATTITUDE_LIMIT_DEG, MavlinkInterface


def horizontal_distance_m(a: GeoPointWGS84, b: GeoPointWGS84) -> float:
    m_lat = meters_per_deg_lat(0.5 * (a.latitude_deg + b.latitude_deg))
    m_lon = meters_per_deg_lon(0.5 * (a.latitude_deg + b.latitude_deg))
    dn = (b.latitude_deg - a.latitude_deg) * m_lat
    de = (b.longitude_deg - a.longitude_deg) * m_lon
    return float((dn * dn + de * de) ** 0.5)


@dataclass
class MissionConfig:
    """FSM eşikleri (sentetik / saha ayarı)."""

    min_track_confidence: float = 0.70
    track_lock_updates: int = 5
    release_radius_m: float = 25.0
    faw_accept_radius_m: float = 40.0
    verify_hold_s: float = 3.0
    go_around_reenter_search: bool = True
    corridor_half_width_m: float = 75.0
    faw_standoff_m: float = FAW_STANDOFF_M
    attitude_limit_deg: float = ATTITUDE_LIMIT_DEG
    max_agl_m: float = 120.0
    telemetry_max_age_s: float = 1.0
    agl_max_age_s: float = 1.0

    def __post_init__(self) -> None:
        for name, value in (
            ("max_agl_m", self.max_agl_m),
            ("telemetry_max_age_s", self.telemetry_max_age_s),
            ("agl_max_age_s", self.agl_max_age_s),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} pozitif ve sonlu olmalı")


class MissionStateMachine:
    """
    Ana görev döngüsü. tick() her kontrol periyodunda bir MissionPacket üretir
    ve gerekiyorsa MAVLink komutu gönderir.
    """

    def __init__(
        self,
        geofence: MissionGeofence,
        mavlink: MavlinkInterface,
        config: Optional[MissionConfig] = None,
    ) -> None:
        self.geofence = geofence
        self.mavlink = mavlink
        self.config = config if config is not None else MissionConfig()
        self.state = MissionState.SEARCH
        self.previous_state = MissionState.SEARCH
        self._sequence = 0
        self._lock_count = 0
        self._plan: Optional[BallisticPlan] = None
        self._locked_target: Optional[GeoPointWGS84] = None
        self._verify_deadline_ns: Optional[int] = None
        self._release_fired = False
        self._last_message = "init"
        self._command_id = ""

    def reset(self) -> None:
        self.previous_state = self.state
        self.state = MissionState.SEARCH
        self._lock_count = 0
        self._plan = None
        self._locked_target = None
        self._verify_deadline_ns = None
        self._release_fired = False
        self._last_message = "reset"
        self.geofence.clear_approach_corridor()

    def _transition(self, new_state: MissionState, message: str) -> None:
        if new_state == self.state:
            self._last_message = message
            return
        self.previous_state = self.state
        self.state = new_state
        self._last_message = message

    def _merge_safety(
        self, telem: TelemetryPacket, geofence_ok: bool
    ) -> SafetyGate:
        base = self.mavlink.evaluate_safety(telem)
        now_ns = time.time_ns()
        source_timestamps = (
            telem.timestamp_ns,
            telem.position_timestamp_ns,
            telem.attitude_timestamp_ns,
            telem.heartbeat_timestamp_ns,
        )
        telemetry_fresh = all(
            timestamp_ns is not None
            and 0 <= now_ns - timestamp_ns
            <= self.config.telemetry_max_age_s * 1e9
            for timestamp_ns in source_timestamps
        )
        agl_age_ns = (
            None
            if telem.agl_timestamp_ns is None
            else now_ns - telem.agl_timestamp_ns
        )
        altitude_ok = (
            telem.agl_altitude_m is not None
            and agl_age_ns is not None
            and 0 <= agl_age_ns <= self.config.agl_max_age_s * 1e9
            and telem.agl_altitude_m <= self.config.max_agl_m
        )
        return SafetyGate(
            flight_mode_ok=base.flight_mode_ok,
            attitude_ok=base.attitude_ok,
            geofence_ok=geofence_ok,
            rc_override_active=base.rc_override_active,
            roll_deg=base.roll_deg,
            pitch_deg=base.pitch_deg,
            telemetry_fresh=telemetry_fresh,
            altitude_ok=altitude_ok,
        )

    def _safety_blocks_engage(self, safety: SafetyGate) -> bool:
        return not safety.all_clear

    def _emit(
        self,
        telem: TelemetryPacket,
        safety: SafetyGate,
        setpoint: Optional[MavlinkSetpoint],
        engage_authorized: bool,
    ) -> MissionPacket:
        self._sequence += 1
        return MissionPacket(
            timestamp_ns=telem.timestamp_ns,
            sequence=self._sequence,
            state=self.state,
            previous_state=self.previous_state,
            safety=safety,
            active_target=self._locked_target,
            setpoint=setpoint,
            referee_events=(),
            command_id=self._command_id,
            engage_authorized=engage_authorized,
            message=self._last_message,
        )

    def _go_around(self, reason: str) -> None:
        self._plan = None
        self._locked_target = None
        self._lock_count = 0
        self._release_fired = False
        self._verify_deadline_ns = None
        self.geofence.clear_approach_corridor()
        self._command_id = f"goa-{self._sequence + 1}"
        if self.mavlink.connected:
            self.mavlink.set_mode_guided()
        self._transition(MissionState.GO_AROUND, reason)

    def _enter_rtl(self, reason: str) -> None:
        self._command_id = f"rtl-{self._sequence + 1}"
        if self.mavlink.connected:
            self.mavlink.set_mode_rtl()
        self._transition(MissionState.RTL, reason)

    def _handle_search(
        self, telem: TelemetryPacket, est: Optional[EstimatorPacket], safety: SafetyGate
    ) -> Optional[MavlinkSetpoint]:
        self._lock_count = 0
        self._plan = None
        if est is None or not est.valid or not est.filter_converged:
            self._last_message = "search: geçerli kestirim yok"
            return None
        if not safety.telemetry_fresh or not safety.altitude_ok:
            self._last_message = "search: telemetri/AGL güvenlik kapısı kapalı"
            return None
        if est.confidence < self.config.min_track_confidence:
            self._last_message = "search: düşük güven"
            return None
        verdict = self.geofence.evaluate(telem.position, est.target_wgs84)
        if not verdict.target_inside or not verdict.uav_inside_ops:
            self._last_message = f"search: {verdict.message}"
            return None
        self._locked_target = est.target_wgs84
        self._lock_count = 1
        self._transition(MissionState.TRACK_LOCK, "search→track_lock")
        return None

    def _handle_track_lock(
        self, telem: TelemetryPacket, est: Optional[EstimatorPacket], safety: SafetyGate
    ) -> Optional[MavlinkSetpoint]:
        if self._safety_blocks_engage(safety):
            self._go_around("track_lock: güvenlik ihlali")
            return None
        if est is None or not est.valid:
            self._go_around("track_lock: kestirim kaybı")
            return None
        if est.confidence < self.config.min_track_confidence:
            self._go_around("track_lock: güven düştü")
            return None
        verdict = self.geofence.evaluate(telem.position, est.target_wgs84)
        if not verdict.target_inside:
            self._go_around(f"track_lock: {verdict.message}")
            return None
        self._locked_target = est.target_wgs84
        self._lock_count += 1
        if self._lock_count >= self.config.track_lock_updates:
            self._transition(MissionState.APPROACH_PLAN, "track_lock→approach_plan")
        else:
            self._last_message = f"track_lock: {self._lock_count}/{self.config.track_lock_updates}"
        return None

    def _handle_approach_plan(
        self, telem: TelemetryPacket, est: Optional[EstimatorPacket], safety: SafetyGate
    ) -> Optional[MavlinkSetpoint]:
        if self._safety_blocks_engage(safety):
            self._go_around("approach_plan: güvenlik ihlali")
            return None
        target = self._locked_target
        if target is None and est is not None and est.valid:
            target = est.target_wgs84
            self._locked_target = target
        if target is None:
            self._go_around("approach_plan: hedef yok")
            return None

        height = max(telem.relative_alt_m, 1.0)
        plan = plan_ballistic_engagement(
            target=target,
            airspeed_mps=telem.airspeed_mps,
            heading_deg=telem.heading_deg,
            groundspeed_north_mps=telem.velocity_ned_mps.north,
            groundspeed_east_mps=telem.velocity_ned_mps.east,
            height_agl_m=height,
            faw_standoff_m=self.config.faw_standoff_m,
        )
        try:
            self.geofence.set_approach_corridor(
                faw=plan.final_approach_waypoint,
                release_point=plan.release_point,
                target=target,
            )
        except ValueError as exc:
            self._go_around(f"approach_plan: koridor hata ({exc})")
            return None

        approach_pts = (plan.final_approach_waypoint, plan.release_point, target)
        verdict = self.geofence.evaluate(telem.position, target, approach_pts)
        if not verdict.ok_for_engage:
            self._go_around(f"approach_plan: {verdict.message}")
            return None

        self._plan = plan
        setpoint = MavlinkSetpoint(
            target=GeoPointWGS84(
                latitude_deg=plan.final_approach_waypoint.latitude_deg,
                longitude_deg=plan.final_approach_waypoint.longitude_deg,
                altitude_m=max(telem.relative_alt_m, 50.0),
            ),
            groundspeed_mps=max(telem.groundspeed_mps, 18.0),
            yaw_deg=telem.heading_deg,
            accept_radius_m=self.config.faw_accept_radius_m,
        )
        self._command_id = f"faw-{self._sequence + 1}"
        if self.mavlink.connected:
            self.mavlink.set_mode_guided()
            self.mavlink.send_guided_waypoint(setpoint)
        self._transition(MissionState.FINAL_RUN, "approach_plan→final_run (FAW)")
        return setpoint

    def _handle_final_run(
        self, telem: TelemetryPacket, safety: SafetyGate
    ) -> Optional[MavlinkSetpoint]:
        if self._safety_blocks_engage(safety):
            self._go_around("final_run: güvenlik ihlali")
            return None
        if self._plan is None or self._locked_target is None:
            self._go_around("final_run: plan yok")
            return None

        approach_pts = (
            self._plan.final_approach_waypoint,
            self._plan.release_point,
            self._locked_target,
        )
        verdict = self.geofence.evaluate(
            telem.position, self._locked_target, approach_pts
        )
        if not verdict.uav_inside_ops or not verdict.approach_inside:
            self._go_around(f"final_run: {verdict.message}")
            return None

        dist_release = horizontal_distance_m(telem.position, self._plan.release_point)
        dist_faw = horizontal_distance_m(
            telem.position, self._plan.final_approach_waypoint
        )

        if dist_release <= self.config.release_radius_m:
            self._transition(MissionState.RELEASE, "final_run→release")
            return MavlinkSetpoint(
                target=GeoPointWGS84(
                    latitude_deg=self._plan.release_point.latitude_deg,
                    longitude_deg=self._plan.release_point.longitude_deg,
                    altitude_m=max(telem.relative_alt_m, 30.0),
                ),
                accept_radius_m=self.config.release_radius_m,
            )

        # FAW'a ulaştıysa Release Point'e geç
        if dist_faw <= self.config.faw_accept_radius_m:
            setpoint = MavlinkSetpoint(
                target=GeoPointWGS84(
                    latitude_deg=self._plan.release_point.latitude_deg,
                    longitude_deg=self._plan.release_point.longitude_deg,
                    altitude_m=max(telem.relative_alt_m, 30.0),
                ),
                groundspeed_mps=max(telem.groundspeed_mps, 18.0),
                accept_radius_m=self.config.release_radius_m,
            )
            self._command_id = f"relwp-{self._sequence + 1}"
            if self.mavlink.connected:
                self.mavlink.send_guided_waypoint(setpoint)
            self._last_message = f"final_run: release'e {dist_release:.1f} m"
            return setpoint

        setpoint = MavlinkSetpoint(
            target=GeoPointWGS84(
                latitude_deg=self._plan.final_approach_waypoint.latitude_deg,
                longitude_deg=self._plan.final_approach_waypoint.longitude_deg,
                altitude_m=max(telem.relative_alt_m, 50.0),
            ),
            accept_radius_m=self.config.faw_accept_radius_m,
        )
        self._last_message = f"final_run: FAW'a {dist_faw:.1f} m"
        return setpoint

    def _handle_release(
        self, telem: TelemetryPacket, safety: SafetyGate
    ) -> Optional[MavlinkSetpoint]:
        # Salım yalnızca all_clear iken
        if self._safety_blocks_engage(safety):
            self._go_around("release: güvenlik — servo yok")
            return None
        if self._plan is None:
            self._go_around("release: plan yok")
            return None

        if not self._release_fired:
            self._command_id = f"servo9-{self._sequence + 1}"
            if self.mavlink.connected:
                self.mavlink.trigger_release_servo_aux9()
            self._release_fired = True
            self._verify_deadline_ns = telem.timestamp_ns + int(
                self.config.verify_hold_s * 1e9
            )
            self._transition(MissionState.VERIFY, "release→verify (AUX9 PWM 2000)")
        return None

    def _handle_verify(self, telem: TelemetryPacket, safety: SafetyGate) -> None:
        _ = safety
        deadline = self._verify_deadline_ns
        if deadline is None:
            self._verify_deadline_ns = telem.timestamp_ns + int(
                self.config.verify_hold_s * 1e9
            )
            self._last_message = "verify: süre başlatıldı"
            return
        if telem.timestamp_ns >= deadline:
            self._enter_rtl("verify→rtl")
        else:
            remain = (deadline - telem.timestamp_ns) / 1e9
            self._last_message = f"verify: {remain:.1f}s"

    def _handle_go_around(self, telem: TelemetryPacket) -> None:
        _ = telem
        if self.config.go_around_reenter_search:
            self._transition(MissionState.SEARCH, "go_around→search")
            self._lock_count = 0
            self._plan = None
            self._locked_target = None
            self._release_fired = False
        else:
            self._enter_rtl("go_around→rtl")

    def tick(
        self,
        telem: TelemetryPacket,
        estimator: Optional[EstimatorPacket] = None,
    ) -> MissionPacket:
        """
        Ana kontrol döngüsü — her çağrıda güvenlik + FSM adımı.
        """
        # Ring-buffer senkronu (sentetik veya canlı)
        self.mavlink.push_telemetry(telem)

        target_for_fence = self._locked_target
        if target_for_fence is None and estimator is not None and estimator.valid:
            target_for_fence = estimator.target_wgs84

        approach_pts = ()
        if self._plan is not None and self._locked_target is not None:
            approach_pts = (
                self._plan.final_approach_waypoint,
                self._plan.release_point,
                self._locked_target,
            )

        verdict = self.geofence.evaluate(
            telem.position, target_for_fence, approach_pts
        )
        safety = self._merge_safety(telem, geofence_ok=verdict.uav_inside_ops)
        # Engage aşamalarında yaklaşma ihlali de unsafe
        if self.state in (
            MissionState.APPROACH_PLAN,
            MissionState.FINAL_RUN,
            MissionState.RELEASE,
        ):
            safety = SafetyGate(
                flight_mode_ok=safety.flight_mode_ok,
                attitude_ok=safety.attitude_ok,
                geofence_ok=verdict.ok_for_engage if target_for_fence else verdict.uav_inside_ops,
                rc_override_active=safety.rc_override_active,
                roll_deg=safety.roll_deg,
                pitch_deg=safety.pitch_deg,
                telemetry_fresh=safety.telemetry_fresh,
                altitude_ok=safety.altitude_ok,
            )

        setpoint: Optional[MavlinkSetpoint] = None
        release_this_tick = False

        if self.state == MissionState.SEARCH:
            setpoint = self._handle_search(telem, estimator, safety)
        elif self.state == MissionState.TRACK_LOCK:
            setpoint = self._handle_track_lock(telem, estimator, safety)
        elif self.state == MissionState.APPROACH_PLAN:
            setpoint = self._handle_approach_plan(telem, estimator, safety)
        elif self.state == MissionState.FINAL_RUN:
            setpoint = self._handle_final_run(telem, safety)
        elif self.state == MissionState.RELEASE:
            before = self._release_fired
            setpoint = self._handle_release(telem, safety)
            release_this_tick = (not before) and self._release_fired
        elif self.state == MissionState.VERIFY:
            self._handle_verify(telem, safety)
        elif self.state == MissionState.GO_AROUND:
            self._handle_go_around(telem)
        elif self.state == MissionState.RTL:
            self._last_message = "rtl: aktif"
        else:
            self._go_around(f"bilinmeyen durum: {self.state}")

        # engage_authorized yalnızca salım tick'inde ve all_clear iken
        engage = release_this_tick and safety.all_clear

        return self._emit(telem, safety, setpoint, engage_authorized=engage)


def run_control_loop_once(
    fsm: MissionStateMachine,
    telem: TelemetryPacket,
    estimator: Optional[EstimatorPacket] = None,
) -> MissionPacket:
    """Tek adımlık ana döngü sarmalayıcısı (test / dış scheduler)."""
    return fsm.tick(telem, estimator)
