"""
comms_gateway — Hakem sunucusu istemcisi + yerel arıza dayanımlı kuyruk.

- Sürekli JSON telemetri (HTTP POST)
- Kilitlenme / bırakma anında bbox çizili JPEG HTTP POST
- Bağlantı kopunca SQLite + JSON yan kuyruk; gelince flush
"""

from __future__ import annotations

import json
import sqlite3
import threading
import time
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np
import requests

from common.data_contracts import (
    Detection2D,
    MissionPacket,
    MissionState,
    TelemetryPacket,
    VisionPacket,
)


class QueueItemType(str, Enum):
    TELEMETRY = "telemetry"
    PHOTO = "photo"
    EVENT = "event"


@dataclass(frozen=True)
class RefereeEndpoints:
    base_url: str
    telemetry_path: str = "/api/telemetry"
    photo_path: str = "/api/photo"
    event_path: str = "/api/event"
    timeout_s: float = 3.0

    def url(self, path: str) -> str:
        return self.base_url.rstrip("/") + path


def _utc_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace(
        "+00:00", "Z"
    )


def telemetry_to_referee_json(
    telem: TelemetryPacket,
    mission: Optional[MissionPacket] = None,
    extra: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Hakem sunucusuna gidecek sürekli telemetri JSON gövdesi."""
    body: Dict[str, Any] = {
        "schema": "ihailos.telemetry.v1",
        "sent_at_utc": _utc_iso(),
        "timestamp_ns": telem.timestamp_ns,
        "sequence": telem.sequence,
        "position": {
            "lat": telem.position.latitude_deg,
            "lon": telem.position.longitude_deg,
            "alt_m": telem.position.altitude_m,
            "relative_alt_m": telem.relative_alt_m,
            "agl_altitude_m": telem.agl_altitude_m,
        },
        "attitude_deg": {
            "roll": telem.roll_deg,
            "pitch": telem.pitch_deg,
            "yaw": float(np.degrees(telem.attitude.yaw)),
        },
        "velocity_ned_mps": {
            "north": telem.velocity_ned_mps.north,
            "east": telem.velocity_ned_mps.east,
            "down": telem.velocity_ned_mps.down,
        },
        "airspeed_mps": telem.airspeed_mps,
        "groundspeed_mps": telem.groundspeed_mps,
        "heading_deg": telem.heading_deg,
        "flight_mode": telem.flight_mode.value,
        "armed": telem.armed,
        "gps_fix_num": telem.gps_fix_num,
        "satellites_visible": telem.satellites_visible,
        "battery": {
            "voltage_v": telem.battery_voltage_v,
            "remaining_pct": telem.battery_remaining_pct,
        },
    }
    if mission is not None:
        body["mission"] = {
            "state": mission.state.value,
            "previous_state": mission.previous_state.value,
            "engage_authorized": mission.engage_authorized,
            "message": mission.message,
            "command_id": mission.command_id,
            "safety": {
                "flight_mode_ok": mission.safety.flight_mode_ok,
                "attitude_ok": mission.safety.attitude_ok,
                "geofence_ok": mission.safety.geofence_ok,
                "rc_override_active": mission.safety.rc_override_active,
                "telemetry_fresh": mission.safety.telemetry_fresh,
                "altitude_ok": mission.safety.altitude_ok,
                "all_clear": mission.safety.all_clear,
            },
        }
        if mission.active_target is not None:
            body["mission"]["active_target"] = {
                "lat": mission.active_target.latitude_deg,
                "lon": mission.active_target.longitude_deg,
                "alt_m": mission.active_target.altitude_m,
            }
    if extra:
        body["extra"] = extra
    return body


def encode_jpeg_bgr(frame_bgr: np.ndarray, quality: int = 90) -> bytes:
    if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
        raise ValueError("frame_bgr HxWx3 olmalı")
    q = int(np.clip(quality, 50, 100))
    ok, buf = cv2.imencode(".jpg", frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), q])
    if not ok:
        raise RuntimeError("JPEG encode başarısız")
    return bytes(buf)


class LocalOutboxQueue:
    """
    SQLite kuyruk + JSON yan depo (payload okunabilirliği).

    internet yokken enqueue; bağlantı gelince dequeue + gönderim.
    """

    def __init__(self, root_dir: str | Path) -> None:
        self.root = Path(root_dir)
        self.root.mkdir(parents=True, exist_ok=True)
        self.db_path = self.root / "outbox.sqlite3"
        self.json_dir = self.root / "json"
        self.blob_dir = self.root / "blobs"
        self.json_dir.mkdir(exist_ok=True)
        self.blob_dir.mkdir(exist_ok=True)
        self._lock = threading.RLock()
        self._init_db()

    def _connect(self) -> sqlite3.Connection:
        conn = sqlite3.connect(str(self.db_path), timeout=30.0, check_same_thread=False)
        conn.row_factory = sqlite3.Row
        return conn

    def _init_db(self) -> None:
        with self._lock:
            conn = self._connect()
            try:
                conn.execute(
                    """
                    CREATE TABLE IF NOT EXISTS outbox (
                        id TEXT PRIMARY KEY,
                        item_type TEXT NOT NULL,
                        created_at_utc TEXT NOT NULL,
                        attempts INTEGER NOT NULL DEFAULT 0,
                        last_error TEXT,
                        json_path TEXT NOT NULL,
                        blob_path TEXT,
                        endpoint_path TEXT NOT NULL
                    )
                    """
                )
                conn.execute(
                    "CREATE INDEX IF NOT EXISTS idx_outbox_created ON outbox(created_at_utc)"
                )
                conn.commit()
            finally:
                conn.close()

    def size(self) -> int:
        with self._lock:
            conn = self._connect()
            try:
                row = conn.execute("SELECT COUNT(*) AS c FROM outbox").fetchone()
                return int(row["c"])
            finally:
                conn.close()

    def enqueue(
        self,
        item_type: QueueItemType,
        endpoint_path: str,
        payload: Dict[str, Any],
        blob: Optional[bytes] = None,
        blob_suffix: str = ".jpg",
    ) -> str:
        item_id = str(uuid.uuid4())
        created = _utc_iso()
        json_path = self.json_dir / f"{item_id}.json"
        blob_path: Optional[Path] = None

        with self._lock:
            json_path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            if blob is not None:
                blob_path = self.blob_dir / f"{item_id}{blob_suffix}"
                blob_path.write_bytes(blob)

            conn = self._connect()
            try:
                conn.execute(
                    """
                    INSERT INTO outbox (
                        id, item_type, created_at_utc, attempts, last_error,
                        json_path, blob_path, endpoint_path
                    ) VALUES (?, ?, ?, 0, NULL, ?, ?, ?)
                    """,
                    (
                        item_id,
                        item_type.value,
                        created,
                        str(json_path),
                        str(blob_path) if blob_path else None,
                        endpoint_path,
                    ),
                )
                conn.commit()
            finally:
                conn.close()
        return item_id

    def peek_batch(self, limit: int = 16) -> Tuple[sqlite3.Row, ...]:
        if limit < 1:
            raise ValueError("limit >= 1")
        with self._lock:
            conn = self._connect()
            try:
                rows = conn.execute(
                    """
                    SELECT * FROM outbox
                    ORDER BY created_at_utc ASC
                    LIMIT ?
                    """,
                    (int(limit),),
                ).fetchall()
                return tuple(rows)
            finally:
                conn.close()

    def mark_failure(self, item_id: str, error: str) -> None:
        with self._lock:
            conn = self._connect()
            try:
                conn.execute(
                    """
                    UPDATE outbox
                    SET attempts = attempts + 1, last_error = ?
                    WHERE id = ?
                    """,
                    (error[:2000], item_id),
                )
                conn.commit()
            finally:
                conn.close()

    def ack(self, item_id: str) -> None:
        with self._lock:
            conn = self._connect()
            try:
                row = conn.execute(
                    "SELECT json_path, blob_path FROM outbox WHERE id = ?",
                    (item_id,),
                ).fetchone()
                if row is not None:
                    jp = Path(row["json_path"])
                    if jp.exists():
                        jp.unlink()
                    if row["blob_path"]:
                        bp = Path(row["blob_path"])
                        if bp.exists():
                            bp.unlink()
                conn.execute("DELETE FROM outbox WHERE id = ?", (item_id,))
                conn.commit()
            finally:
                conn.close()

    def load_payload(self, row: sqlite3.Row) -> Tuple[Dict[str, Any], Optional[bytes]]:
        payload = json.loads(Path(row["json_path"]).read_text(encoding="utf-8"))
        blob: Optional[bytes] = None
        if row["blob_path"]:
            blob = Path(row["blob_path"]).read_bytes()
        return payload, blob


class RefereeClient:
    """
    Arıza dayanımlı hakem istemcisi.

    push_telemetry: sürekli JSON
    submit_lock_photo / submit_release_photo: bbox çizili JPEG + meta
    flush_outbox: kuyruğu boşalt
    """

    def __init__(
        self,
        endpoints: RefereeEndpoints,
        queue_dir: str | Path,
        session: Optional[requests.Session] = None,
        team_id: str = "IHAILOS",
        telemetry_min_interval_s: float = 0.2,
    ) -> None:
        if telemetry_min_interval_s < 0.0:
            raise ValueError("telemetry_min_interval_s negatif olamaz")
        self.endpoints = endpoints
        self.queue = LocalOutboxQueue(queue_dir)
        self.session = session if session is not None else requests.Session()
        self.team_id = team_id
        self.telemetry_min_interval_s = float(telemetry_min_interval_s)
        self._last_telemetry_mono = 0.0
        self._online = True
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._flush_thread: Optional[threading.Thread] = None

    @property
    def is_online(self) -> bool:
        return self._online

    @property
    def pending_count(self) -> int:
        return self.queue.size()

    def close(self) -> None:
        self.stop_background_flusher()
        self.session.close()

    def start_background_flusher(self, period_s: float = 2.0) -> None:
        if self._flush_thread is not None and self._flush_thread.is_alive():
            return
        if period_s <= 0.0:
            raise ValueError("period_s pozitif olmalı")
        self._stop.clear()

        def _loop() -> None:
            while not self._stop.wait(period_s):
                self.flush_outbox()

        self._flush_thread = threading.Thread(
            target=_loop, name="referee-outbox-flush", daemon=True
        )
        self._flush_thread.start()

    def stop_background_flusher(self) -> None:
        self._stop.set()
        if self._flush_thread is not None:
            self._flush_thread.join(timeout=5.0)
            self._flush_thread = None

    def _post_json(self, path: str, payload: Dict[str, Any]) -> requests.Response:
        url = self.endpoints.url(path)
        return self.session.post(
            url,
            json=payload,
            timeout=self.endpoints.timeout_s,
            headers={"Content-Type": "application/json"},
        )

    def _post_multipart(
        self,
        path: str,
        payload: Dict[str, Any],
        image_bytes: bytes,
        filename: str = "frame.jpg",
    ) -> requests.Response:
        url = self.endpoints.url(path)
        files = {"image": (filename, image_bytes, "image/jpeg")}
        data = {"meta": json.dumps(payload, ensure_ascii=False)}
        return self.session.post(
            url,
            data=data,
            files=files,
            timeout=self.endpoints.timeout_s,
        )

    def _handle_transport(
        self,
        item_type: QueueItemType,
        endpoint_path: str,
        payload: Dict[str, Any],
        blob: Optional[bytes] = None,
    ) -> bool:
        """True = sunucuya gitti; False = kuyruğa alındı."""
        try:
            if blob is None:
                resp = self._post_json(endpoint_path, payload)
            else:
                resp = self._post_multipart(endpoint_path, payload, blob)
            if 200 <= resp.status_code < 300:
                self._online = True
                return True
            raise RuntimeError(f"HTTP {resp.status_code}: {resp.text[:300]}")
        except (requests.RequestException, RuntimeError, OSError) as exc:
            self._online = False
            self.queue.enqueue(
                item_type=item_type,
                endpoint_path=endpoint_path,
                payload=payload,
                blob=blob,
            )
            _ = exc
            return False

    def push_telemetry(
        self,
        telem: TelemetryPacket,
        mission: Optional[MissionPacket] = None,
        force: bool = False,
        extra: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """Sürekli JSON telemetri; başarısızsa outbox."""
        now = time.monotonic()
        with self._lock:
            if (
                not force
                and (now - self._last_telemetry_mono) < self.telemetry_min_interval_s
            ):
                return True
            self._last_telemetry_mono = now

        payload = telemetry_to_referee_json(telem, mission, extra)
        payload["team_id"] = self.team_id
        return self._handle_transport(
            QueueItemType.TELEMETRY,
            self.endpoints.telemetry_path,
            payload,
        )

    def submit_annotated_photo(
        self,
        event: str,
        frame_bgr: np.ndarray,
        detections: Tuple[Detection2D, ...],
        telem: Optional[TelemetryPacket] = None,
        mission: Optional[MissionPacket] = None,
        vision: Optional[VisionPacket] = None,
    ) -> bool:
        """
        Bounding box çizili fotoğrafı HTTP POST eder.
        event: 'track_lock' | 'release' | özel etiket
        """
        from vision_engine.detector import draw_detections_bgr

        annotated = draw_detections_bgr(frame_bgr, detections)
        jpeg = encode_jpeg_bgr(annotated)
        meta: Dict[str, Any] = {
            "schema": "ihailos.photo.v1",
            "team_id": self.team_id,
            "event": event,
            "sent_at_utc": _utc_iso(),
            "detections": [
                {
                    "class": d.class_name.value,
                    "confidence": d.confidence,
                    "bbox_xyxy": [d.bbox.x1, d.bbox.y1, d.bbox.x2, d.bbox.y2],
                }
                for d in detections
            ],
        }
        if telem is not None:
            meta["telemetry"] = telemetry_to_referee_json(telem, mission)
        if vision is not None:
            meta["vision"] = {
                "frame_id": vision.frame_id,
                "timestamp_ns": vision.timestamp_ns,
                "model_name": vision.model_name,
            }
        if mission is not None:
            meta["mission_state"] = mission.state.value
            meta["command_id"] = mission.command_id

        return self._handle_transport(
            QueueItemType.PHOTO,
            self.endpoints.photo_path,
            meta,
            blob=jpeg,
        )

    def submit_lock_photo(
        self,
        frame_bgr: np.ndarray,
        detections: Tuple[Detection2D, ...],
        telem: Optional[TelemetryPacket] = None,
        mission: Optional[MissionPacket] = None,
        vision: Optional[VisionPacket] = None,
    ) -> bool:
        return self.submit_annotated_photo(
            "track_lock", frame_bgr, detections, telem, mission, vision
        )

    def submit_release_photo(
        self,
        frame_bgr: np.ndarray,
        detections: Tuple[Detection2D, ...],
        telem: Optional[TelemetryPacket] = None,
        mission: Optional[MissionPacket] = None,
        vision: Optional[VisionPacket] = None,
    ) -> bool:
        return self.submit_annotated_photo(
            "release", frame_bgr, detections, telem, mission, vision
        )

    def notify_mission_edge(
        self,
        mission: MissionPacket,
        telem: TelemetryPacket,
        frame_bgr: Optional[np.ndarray] = None,
        detections: Tuple[Detection2D, ...] = (),
    ) -> None:
        """
        FSM kenar olayları: TRACK_LOCK / RELEASE fotoğraf + event JSON.
        """
        if mission.state == MissionState.TRACK_LOCK and frame_bgr is not None:
            self.submit_lock_photo(frame_bgr, detections, telem, mission)
        elif mission.state == MissionState.RELEASE and frame_bgr is not None:
            self.submit_release_photo(frame_bgr, detections, telem, mission)

        event_payload = {
            "schema": "ihailos.event.v1",
            "team_id": self.team_id,
            "sent_at_utc": _utc_iso(),
            "state": mission.state.value,
            "previous_state": mission.previous_state.value,
            "message": mission.message,
            "command_id": mission.command_id,
            "engage_authorized": mission.engage_authorized,
            "telemetry_seq": telem.sequence,
        }
        self._handle_transport(
            QueueItemType.EVENT,
            self.endpoints.event_path,
            event_payload,
        )

    def flush_outbox(self, max_items: int = 32) -> int:
        """Kuyruktaki öğeleri sırayla tekrar dener; başarılı sayısı döner."""
        sent = 0
        for row in self.queue.peek_batch(limit=max_items):
            try:
                payload, blob = self.queue.load_payload(row)
                path = str(row["endpoint_path"])
                if blob is None:
                    resp = self._post_json(path, payload)
                else:
                    resp = self._post_multipart(path, payload, blob)
                if 200 <= resp.status_code < 300:
                    self.queue.ack(str(row["id"]))
                    sent += 1
                    self._online = True
                else:
                    self.queue.mark_failure(
                        str(row["id"]), f"HTTP {resp.status_code}"
                    )
                    self._online = False
                    break
            except (requests.RequestException, OSError, json.JSONDecodeError) as exc:
                self.queue.mark_failure(str(row["id"]), str(exc))
                self._online = False
                break
        return sent
