"""Open-vocabulary object detector backed by YOLO-World (ultralytics).

The real backend behind ``perception_node._make_detector(name="yolo_world"|"openvocab")``.
Unlike ``MockDetector`` it detects an arbitrary, caller-supplied vocabulary of class names
(no fixed label set, no retraining) — YOLO-World matches text prompts against image regions.

``ultralytics``/``torch`` are imported lazily in ``__init__`` so the package imports (and the
mock pipeline + unit tests run) with no ML dependency or GPU. The result-parsing logic is kept
in the pure module-level ``_parse_results`` so it can be unit-tested with a fake Results object.
"""
from __future__ import annotations

from typing import List, Optional

import numpy as np

from semantic_perception.detector import Detection2D, Detector

# Small open-vocab checkpoint — ~25 MB, auto-downloaded by ultralytics on first use.
# 's' (small) keeps it comfortably within the 8 GB-VRAM budget alongside the other backends.
DEFAULT_WEIGHTS = "yolov8s-worldv2.pt"


def _to_numpy(arr) -> np.ndarray:
    """Tensor (possibly on GPU) or array-like -> CPU numpy array."""
    if hasattr(arr, "cpu"):          # torch.Tensor
        arr = arr.cpu().numpy()
    return np.asarray(arr)


def _parse_results(results, vocabulary: List[str]) -> List[Detection2D]:
    """Map an ultralytics predict() result list into Detection2D boxes.

    ``results`` is the list returned by ``model.predict(...)`` (one entry per image; we use
    the first). Class indices map into ``vocabulary`` (the order passed to ``set_classes``);
    an out-of-range index falls back to its string form rather than dropping the detection.
    """
    if not results:
        return []
    boxes = getattr(results[0], "boxes", None)
    if boxes is None:
        return []
    xyxy = _to_numpy(boxes.xyxy).reshape(-1, 4)
    conf = _to_numpy(boxes.conf).reshape(-1)
    cls = _to_numpy(boxes.cls).reshape(-1).astype(int)

    dets: List[Detection2D] = []
    for (x1, y1, x2, y2), score, idx in zip(xyxy, conf, cls):
        label = vocabulary[idx] if 0 <= idx < len(vocabulary) else str(idx)
        dets.append(Detection2D(
            label=label, score=float(score),
            x=int(x1), y=int(y1),
            width=int(x2 - x1), height=int(y2 - y1)))
    return dets


class OpenVocabDetector(Detector):
    """YOLO-World detector restricted to a fixed vocabulary of text class prompts."""

    def __init__(self, vocabulary: List[str], confidence_threshold: float,
                 weights: str = DEFAULT_WEIGHTS, model=None):
        self.vocabulary = list(vocabulary)
        self.confidence_threshold = float(confidence_threshold)
        if model is None:
            from ultralytics import YOLO  # lazy: only when a real detector is selected
            model = YOLO(weights)
        self.model = model
        # Constrain the open-vocab head to our prompts; cls indices then map into `vocabulary`.
        self.model.set_classes(self.vocabulary)

    def detect(self, rgb: np.ndarray) -> List[Detection2D]:
        # `rgb` is actually BGR (the node hands us a bgr8 frame); ultralytics accepts ndarrays.
        results = self.model.predict(
            rgb, conf=self.confidence_threshold, verbose=False)
        return _parse_results(results, self.vocabulary)
