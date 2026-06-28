"""Pluggable 2D object detectors. Pure (no ROS); real backends live elsewhere.

The detector's only job is to turn an RGB image into labelled 2D boxes; deprojection
to 3D and map-frame transforms are the node's responsibility. MockDetector lets the
whole perception pipeline be exercised with no ML dependency or GPU.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List

import numpy as np


@dataclass
class Detection2D:
    """An axis-aligned detection in image pixels."""

    label: str
    score: float
    x: int
    y: int
    width: int
    height: int


class Detector(ABC):
    """Turns an HxWx3 RGB image into a list of 2D detections."""

    @abstractmethod
    def detect(self, rgb: np.ndarray) -> List[Detection2D]:
        ...


class MockDetector(Detector):
    """Deterministic detector: one centred box covering ``box_frac`` of the frame.

    Useful for validating deprojection + TF end-to-end (point an object at the image
    centre and the detection's 3D position should land on it).
    """

    def __init__(self, label: str = "object", box_frac: float = 0.33, score: float = 1.0):
        self.label = label
        self.box_frac = box_frac
        self.score = score

    def detect(self, rgb: np.ndarray) -> List[Detection2D]:
        h, w = rgb.shape[:2]
        bw = int(w * self.box_frac)
        bh = int(h * self.box_frac)
        x = (w - bw) // 2
        y = (h - bh) // 2
        return [Detection2D(self.label, self.score, x, y, bw, bh)]
