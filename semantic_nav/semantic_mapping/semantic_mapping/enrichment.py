"""Batch enrichment of the semantic map: turn retained image crops into rich
descriptions and embeddings, then cluster entities into regions.

The AI backends are pluggable and mockable (mirrors ``semantic_perception.detector``):
the mocks make the whole enrich -> persist -> query pipeline runnable with no GPU,
network, or model download. Real backends (ollama VLM, CLIP) are added later behind
the same ``Describer`` / ``Embedder`` interfaces.

``Embedder`` exposes both ``embed_image`` (used here in Phase 1) and ``embed_text``
(used by the Phase-2 query server), so a real CLIP-style model can place crops and
text queries in one shared space.
"""
from __future__ import annotations

import hashlib
from abc import ABC, abstractmethod
from typing import Callable, Dict, Optional

import numpy as np

from semantic_store.regions import assign_regions


class Describer(ABC):
    """Turns an HxWx3 BGR crop into a natural-language description."""

    @abstractmethod
    def describe(self, crop_bgr: np.ndarray) -> str:
        ...


class Embedder(ABC):
    """Embeds crops and text into one shared vector space (CLIP-style)."""

    @abstractmethod
    def embed_image(self, crop_bgr: np.ndarray) -> np.ndarray:
        ...

    @abstractmethod
    def embed_text(self, text: str) -> np.ndarray:
        ...


def _unit_vector_from_bytes(data: bytes, dim: int) -> np.ndarray:
    """Deterministic, content-dependent, L2-normalized vector seeded from ``data``."""
    seed = int.from_bytes(hashlib.sha256(data).digest()[:8], "big")
    vec = np.random.default_rng(seed).standard_normal(dim)
    norm = float(np.linalg.norm(vec))
    return vec / norm if norm > 0.0 else vec


class MockDescriber(Describer):
    """Deterministic placeholder describer: no model, just crop statistics."""

    def describe(self, crop_bgr: np.ndarray) -> str:
        h, w = crop_bgr.shape[:2]
        mean = crop_bgr.reshape(-1, crop_bgr.shape[-1]).mean(axis=0)
        bgr = ", ".join(f"{c:.0f}" for c in mean)
        return f"a {w}x{h} object crop (mean BGR {bgr})"


class MockEmbedder(Embedder):
    """Deterministic hash-based embedder. Vectors are content-dependent and
    normalized, but image and text spaces are NOT aligned (a real CLIP is)."""

    def __init__(self, dim: int = 512):
        self.dim = dim

    def embed_image(self, crop_bgr: np.ndarray) -> np.ndarray:
        return _unit_vector_from_bytes(np.ascontiguousarray(crop_bgr).tobytes(), self.dim)

    def embed_text(self, text: str) -> np.ndarray:
        return _unit_vector_from_bytes(("txt:" + text).encode("utf-8"), self.dim)


def enrich_store(
    store,
    crops_by_id: Dict[int, np.ndarray],
    *,
    describer: Optional[Describer] = None,
    embedder: Optional[Embedder] = None,
    region_eps: float = 0.0,
    on_error: Optional[Callable[[int, str, Exception], None]] = None,
) -> int:
    """Enrich entities that have a retained crop, then optionally cluster regions.

    Returns the number of entities enriched (those with a crop where at least one
    backend succeeded). Entities without a crop are left untouched.

    Each backend call is isolated: a real describer/embedder that throws on one
    crop (e.g. a transient model error or a malformed crop) must not abort the
    whole batch, since the caller persists the store afterward and a single
    failure would otherwise leave the map unsaved. Failures are reported via the
    optional ``on_error(entity_id, stage, exc)`` callback ("describe"/"embed");
    the module stays pure (no logging side effects of its own).
    """
    enriched = 0
    for e in store.entities:
        crop = crops_by_id.get(e.id)
        if crop is None or (describer is None and embedder is None):
            continue
        ok = False
        if describer is not None:
            try:
                e.description = describer.describe(crop)
                ok = True
            except Exception as exc:  # noqa: BLE001 - one bad crop must not abort the batch
                if on_error is not None:
                    on_error(e.id, "describe", exc)
        if embedder is not None:
            try:
                e.embedding = embedder.embed_image(crop)
                ok = True
            except Exception as exc:  # noqa: BLE001 - one bad crop must not abort the batch
                if on_error is not None:
                    on_error(e.id, "embed", exc)
        if ok:
            enriched += 1

    if region_eps and region_eps > 0:
        assign_regions(store.entities, eps=region_eps)

    return enriched
