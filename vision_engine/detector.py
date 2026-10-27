"""
vision_engine — OpenCV kare yakalama, UTC timestamp, Ultralytics YOLO, 8-kare debounce.

Üretir: VisionPacket (estimator tüketicisi)
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Protocol, Sequence, Tuple

import cv2
import numpy as np

from common.data_contracts import (
    BoundingBoxXYXY,
    CameraIntrinsics,
    Detection2D,
    TargetClass,
    VisionPacket,
)

DEFAULT_DEBOUNCE_FRAMES = 8
DEFAULT_CONF_THRESHOLD = 0.35
DEFAULT_IOU_MATCH = 0.30


class YoloLikeModel(Protocol):
    """Ultralytics YOLO ile uyumlu minimal arayüz (test için fake model)."""

    def __call__(self, source: Any, **kwargs: Any) -> Any:
        ...


def utc_timestamp_ns() -> int:
    """Kare yakalama anı — UTC nanosecond epoch."""
    return time.time_ns()


def utc_iso8601_from_ns(timestamp_ns: int) -> str:
    dt = datetime.fromtimestamp(timestamp_ns / 1e9, tz=timezone.utc)
    return dt.isoformat(timespec="milliseconds").replace("+00:00", "Z")


def map_class_name(raw: str) -> TargetClass:
    """YOLO sınıf adını TargetClass'a eşler (sentetik/açık etiketler)."""
    key = raw.strip().lower()
    table: Dict[str, TargetClass] = {
        "hss": TargetClass.HSS,
        "hostile": TargetClass.HOSTILE,
        "enemy": TargetClass.HOSTILE,
        "friendly": TargetClass.FRIENDLY,
        "vehicle": TargetClass.VEHICLE,
        "car": TargetClass.VEHICLE,
        "truck": TargetClass.VEHICLE,
        "building": TargetClass.BUILDING,
        "unknown": TargetClass.UNKNOWN,
    }
    return table.get(key, TargetClass.UNKNOWN)


def bbox_iou(a: BoundingBoxXYXY, b: BoundingBoxXYXY) -> float:
    x1 = max(a.x1, b.x1)
    y1 = max(a.y1, b.y1)
    x2 = min(a.x2, b.x2)
    y2 = min(a.y2, b.y2)
    inter_w = max(0.0, x2 - x1)
    inter_h = max(0.0, y2 - y1)
    inter = inter_w * inter_h
    if inter <= 0.0:
        return 0.0
    area_a = a.width * a.height
    area_b = b.width * b.height
    union = area_a + area_b - inter
    if union <= 0.0:
        return 0.0
    return float(inter / union)


def default_intrinsics(width: int, height: int, fx: Optional[float] = None) -> CameraIntrinsics:
    """Kalibrasyon yoksa makul pinhole tahmini (fx ≈ 0.9 * max(w,h))."""
    if width <= 0 or height <= 0:
        raise ValueError("görüntü boyutları pozitif olmalı")
    focal = float(fx) if fx is not None else 0.9 * float(max(width, height))
    return CameraIntrinsics(
        fx=focal,
        fy=focal,
        cx=width * 0.5,
        cy=height * 0.5,
        width_px=width,
        height_px=height,
    )


@dataclass(frozen=True)
class DebounceStatus:
    """8 ardışık kare doğrulama durumu."""

    streak: int
    required: int
    locked: bool
    primary: Optional[Detection2D]

    @property
    def progress(self) -> float:
        return min(1.0, float(self.streak) / float(self.required))


class DetectionDebouncer:
    """
    Aynı sınıf + yeterli IoU ile N ardışık karede doğrulama.
    Kilit kırılırsa sayaç sıfırlanır.
    """

    def __init__(
        self,
        required_frames: int = DEFAULT_DEBOUNCE_FRAMES,
        iou_threshold: float = DEFAULT_IOU_MATCH,
        conf_threshold: float = DEFAULT_CONF_THRESHOLD,
    ) -> None:
        if required_frames < 1:
            raise ValueError("required_frames >= 1 olmalı")
        if not (0.0 < iou_threshold <= 1.0):
            raise ValueError("iou_threshold (0, 1] olmalı")
        if not (0.0 <= conf_threshold <= 1.0):
            raise ValueError("conf_threshold [0, 1] olmalı")
        self.required_frames = int(required_frames)
        self.iou_threshold = float(iou_threshold)
        self.conf_threshold = float(conf_threshold)
        self._history: Deque[Optional[Detection2D]] = deque(maxlen=required_frames)
        self._streak = 0
        self._anchor: Optional[Detection2D] = None
        self._locked = False

    def reset(self) -> None:
        self._history.clear()
        self._streak = 0
        self._anchor = None
        self._locked = False

    def _pick_primary(self, detections: Sequence[Detection2D]) -> Optional[Detection2D]:
        pool = [d for d in detections if d.confidence >= self.conf_threshold]
        if not pool:
            return None
        return max(pool, key=lambda d: d.confidence)

    def update(self, detections: Sequence[Detection2D]) -> DebounceStatus:
        primary = self._pick_primary(detections)
        self._history.append(primary)

        if primary is None:
            self._streak = 0
            self._anchor = None
            self._locked = False
            return DebounceStatus(0, self.required_frames, False, None)

        if self._anchor is None:
            self._anchor = primary
            self._streak = 1
            self._locked = self._streak >= self.required_frames
            return DebounceStatus(self._streak, self.required_frames, self._locked, primary)

        same_class = primary.class_name == self._anchor.class_name
        iou = bbox_iou(primary.bbox, self._anchor.bbox)
        if same_class and iou >= self.iou_threshold:
            self._streak = min(self._streak + 1, self.required_frames)
            # Anchor'ı yumuşat: güncel kutuyu tut
            self._anchor = primary
        else:
            self._anchor = primary
            self._streak = 1

        self._locked = self._streak >= self.required_frames
        return DebounceStatus(
            streak=self._streak,
            required=self.required_frames,
            locked=self._locked,
            primary=primary if self._locked else None,
        )

    @property
    def is_locked(self) -> bool:
        return self._locked


def load_yolo_model(model_path: str | Path) -> Any:
    """Ultralytics YOLO / TensorRT .engine yükler."""
    from ultralytics import YOLO  # ağır bağımlılık: yalnızca burada

    path = Path(model_path)
    if not path.exists():
        raise FileNotFoundError(f"YOLO model bulunamadı: {path}")
    return YOLO(str(path))


def _extract_detections_from_yolo_result(
    result: Any,
    conf_threshold: float,
) -> List[Detection2D]:
    detections: List[Detection2D] = []
    boxes = getattr(result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        return detections

    names = getattr(result, "names", None)
    if names is None:
        names = {}

    xyxy = boxes.xyxy
    confs = boxes.conf
    clss = boxes.cls
    if hasattr(xyxy, "cpu"):
        xyxy = xyxy.cpu().numpy()
        confs = confs.cpu().numpy()
        clss = clss.cpu().numpy()
    else:
        xyxy = np.asarray(xyxy, dtype=np.float64)
        confs = np.asarray(confs, dtype=np.float64)
        clss = np.asarray(clss, dtype=np.float64)

    for i in range(len(xyxy)):
        conf = float(confs[i])
        if conf < conf_threshold:
            continue
        class_id = int(clss[i])
        if isinstance(names, dict):
            raw_name = str(names.get(class_id, "unknown"))
        else:
            raw_name = "unknown"
        x1, y1, x2, y2 = (float(v) for v in xyxy[i][:4])
        detections.append(
            Detection2D(
                detection_id=i,
                class_id=class_id,
                class_name=map_class_name(raw_name),
                confidence=conf,
                bbox=BoundingBoxXYXY(x1=x1, y1=y1, x2=x2, y2=y2),
            )
        )
    return detections


def draw_detections_bgr(
    frame_bgr: np.ndarray,
    detections: Sequence[Detection2D],
    color: Tuple[int, int, int] = (0, 255, 0),
) -> np.ndarray:
    """Bounding box çizili kopya (hakem fotoğrafı için)."""
    out = frame_bgr.copy()
    for det in detections:
        p1 = (int(det.bbox.x1), int(det.bbox.y1))
        p2 = (int(det.bbox.x2), int(det.bbox.y2))
        cv2.rectangle(out, p1, p2, color, 2)
        label = f"{det.class_name.value}:{det.confidence:.2f}"
        cv2.putText(
            out,
            label,
            (p1[0], max(0, p1[1] - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA,
        )
    return out


@dataclass
class FrameCapture:
    """Tek kare + UTC zaman damgası."""

    frame_bgr: np.ndarray
    timestamp_ns: int
    frame_id: int
    utc_iso: str


class VisionDetector:
    """
    OpenCV kamera → UTC stamp → YOLO inference → 8-kare debounce → VisionPacket.
    """

    def __init__(
        self,
        model: Any,
        camera: CameraIntrinsics,
        source: int | str = 0,
        conf_threshold: float = DEFAULT_CONF_THRESHOLD,
        debounce_frames: int = DEFAULT_DEBOUNCE_FRAMES,
        iou_match: float = DEFAULT_IOU_MATCH,
        model_name: str = "yolo",
        tensorrt_engine: bool = False,
        capture: Optional[cv2.VideoCapture] = None,
    ) -> None:
        if debounce_frames < 1:
            raise ValueError("debounce_frames >= 1")
        self.model = model
        self.camera = camera
        self.source = source
        self.conf_threshold = float(conf_threshold)
        self.model_name = model_name
        self.tensorrt_engine = bool(tensorrt_engine)
        self._cap = capture
        self._owns_capture = capture is None
        self._frame_id = 0
        self._debouncer = DetectionDebouncer(
            required_frames=debounce_frames,
            iou_threshold=iou_match,
            conf_threshold=conf_threshold,
        )
        self._last_frame_bgr: Optional[np.ndarray] = None
        self._last_raw_detections: Tuple[Detection2D, ...] = ()
        self._last_debounce = DebounceStatus(0, debounce_frames, False, None)

    @classmethod
    def from_model_path(
        cls,
        model_path: str | Path,
        camera: Optional[CameraIntrinsics] = None,
        source: int | str = 0,
        **kwargs: Any,
    ) -> VisionDetector:
        path = Path(model_path)
        model = load_yolo_model(path)
        tensorrt = path.suffix.lower() == ".engine"
        cam = camera if camera is not None else default_intrinsics(1280, 720)
        return cls(
            model=model,
            camera=cam,
            source=source,
            model_name=path.name,
            tensorrt_engine=tensorrt,
            **kwargs,
        )

    def open(self) -> None:
        if self._cap is not None and self._cap.isOpened():
            return
        self._cap = cv2.VideoCapture(self.source)
        if not self._cap.isOpened():
            raise RuntimeError(f"kamera/açılamadı: source={self.source}")
        self._owns_capture = True
        w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or self.camera.width_px
        h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or self.camera.height_px
        if w != self.camera.width_px or h != self.camera.height_px:
            self.camera = CameraIntrinsics(
                fx=self.camera.fx,
                fy=self.camera.fy,
                cx=w * 0.5,
                cy=h * 0.5,
                width_px=w,
                height_px=h,
                dist_coeffs=self.camera.dist_coeffs,
            )

    def close(self) -> None:
        if self._cap is not None and self._owns_capture:
            self._cap.release()
        self._cap = None

    def __enter__(self) -> VisionDetector:
        self.open()
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        self.close()

    def grab_frame(self) -> FrameCapture:
        """OpenCV read + yakalama anında UTC timestamp."""
        if self._cap is None or not self._cap.isOpened():
            self.open()
        assert self._cap is not None
        ok, frame = self._cap.read()
        timestamp_ns = utc_timestamp_ns()
        if not ok or frame is None:
            raise RuntimeError("kare okunamadı")
        self._frame_id += 1
        self._last_frame_bgr = frame
        return FrameCapture(
            frame_bgr=frame,
            timestamp_ns=timestamp_ns,
            frame_id=self._frame_id,
            utc_iso=utc_iso8601_from_ns(timestamp_ns),
        )

    def infer_detections(self, frame_bgr: np.ndarray) -> Tuple[List[Detection2D], float]:
        t0 = time.perf_counter()
        results = self.model(frame_bgr, verbose=False, conf=self.conf_threshold)
        latency_ms = (time.perf_counter() - t0) * 1000.0
        if isinstance(results, (list, tuple)):
            if not results:
                return [], float(latency_ms)
            result = results[0]
        else:
            result = results
        dets = _extract_detections_from_yolo_result(result, self.conf_threshold)
        return dets, float(latency_ms)

    def process_frame(
        self,
        frame: FrameCapture,
        sync_telemetry_seq: int = 0,
        emit_only_when_locked: bool = False,
    ) -> VisionPacket:
        """YOLO + debounce; VisionPacket üretir."""
        dets, latency_ms = self.infer_detections(frame.frame_bgr)
        self._last_raw_detections = tuple(dets)
        status = self._debouncer.update(dets)
        self._last_debounce = status

        if emit_only_when_locked and not status.locked:
            out_dets: Tuple[Detection2D, ...] = ()
        elif status.locked and status.primary is not None:
            # Debounce kilidi: doğrulanmış birincil + ham liste
            out_dets = (status.primary,) + tuple(
                d for d in dets if d.detection_id != status.primary.detection_id
            )
        else:
            out_dets = tuple(dets)

        h, w = frame.frame_bgr.shape[:2]
        return VisionPacket(
            timestamp_ns=frame.timestamp_ns,
            frame_id=frame.frame_id,
            sync_telemetry_seq=sync_telemetry_seq,
            image_width=int(w),
            image_height=int(h),
            detections=out_dets,
            camera=self.camera,
            inference_latency_ms=latency_ms,
            model_name=self.model_name,
            tensorrt_engine=self.tensorrt_engine,
        )

    def step(self, sync_telemetry_seq: int = 0) -> VisionPacket:
        """grab → infer → debounce → VisionPacket."""
        frame = self.grab_frame()
        return self.process_frame(frame, sync_telemetry_seq=sync_telemetry_seq)

    @property
    def debounce_status(self) -> DebounceStatus:
        return self._last_debounce

    @property
    def last_frame_bgr(self) -> Optional[np.ndarray]:
        return self._last_frame_bgr

    def annotated_frame(self) -> Optional[np.ndarray]:
        if self._last_frame_bgr is None:
            return None
        dets = self._last_raw_detections
        if self._last_debounce.locked and self._last_debounce.primary is not None:
            dets = (self._last_debounce.primary,)
        return draw_detections_bgr(self._last_frame_bgr, dets)
