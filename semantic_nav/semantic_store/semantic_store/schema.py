"""Plain data records used throughout the store. No ROS, no I/O."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


def as_vec3(v) -> np.ndarray:
    """Coerce ``v`` into a float (3,) array, validating its length."""
    a = np.asarray(v, dtype=float).reshape(-1)
    if a.shape[0] != 3:
        raise ValueError(f"expected a 3-vector, got shape {a.shape}")
    return a


@dataclass
class Observation:
    """One raw, grounded detection at a single instant, expressed in the map frame.

    ``embedding``/``size`` are optional: detection may run before enrichment fills them in.
    """

    label: str
    position: np.ndarray            # (3,) map-frame position
    confidence: float = 1.0
    stamp: float = 0.0              # seconds
    embedding: Optional[np.ndarray] = None
    size: Optional[np.ndarray] = None
    crop_ref: str = ""             # id/path of the saved image crop (for batch enrichment)

    def __post_init__(self) -> None:
        self.position = as_vec3(self.position)
        if self.embedding is not None:
            self.embedding = np.asarray(self.embedding, dtype=float).reshape(-1)
        if self.size is not None:
            self.size = as_vec3(self.size)


@dataclass
class SemanticEntity:
    """A consolidated object fused from one or more observations."""

    id: int
    label: str                     # canonical / primary label
    position: np.ndarray           # (3,) running centroid in the map frame
    confidence: float = 0.0        # aggregate (max) detection confidence
    observation_count: int = 0
    last_seen: float = 0.0
    aliases: List[str] = field(default_factory=list)
    description: str = ""          # filled during batch enrichment (VLM)
    region: str = ""               # filled during region clustering / naming
    embedding: Optional[np.ndarray] = None
    size: Optional[np.ndarray] = None
