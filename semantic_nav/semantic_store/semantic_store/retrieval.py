"""Ranked retrieval over a list of entities.

Filters (region, spatial radius, label substring) narrow the candidate set; then a
single ranking signal orders it — embedding similarity if a query embedding is given,
else proximity to a point, else aggregate confidence.
"""
from __future__ import annotations

from typing import List, Optional, Sequence

import numpy as np

from .association import cosine_similarity
from .schema import SemanticEntity, as_vec3


def _matches_label(entity: SemanticEntity, query: str) -> bool:
    q = query.lower()
    return q in entity.label.lower() or any(q in a.lower() for a in entity.aliases)


def query(
    entities: Sequence[SemanticEntity],
    *,
    text_embedding: Optional[np.ndarray] = None,
    label: Optional[str] = None,
    region: Optional[str] = None,
    near=None,
    radius: Optional[float] = None,
    top_k: Optional[int] = None,
) -> List[SemanticEntity]:
    near_vec = None if near is None else as_vec3(near)
    results = list(entities)

    if label:
        results = [e for e in results if _matches_label(e, label)]
    if region:
        results = [e for e in results if e.region.lower() == region.lower()]
    if near_vec is not None and radius and radius > 0:
        results = [
            e for e in results
            if float(np.linalg.norm(e.position - near_vec)) <= radius
        ]

    if text_embedding is not None:
        te = np.asarray(text_embedding, dtype=float).reshape(-1)
        results.sort(
            key=lambda e: cosine_similarity(e.embedding, te) if e.embedding is not None else -1.0,
            reverse=True,
        )
    elif near_vec is not None:
        results.sort(key=lambda e: float(np.linalg.norm(e.position - near_vec)))
    else:
        results.sort(key=lambda e: (e.confidence, e.observation_count), reverse=True)

    if top_k and top_k > 0:
        results = results[:top_k]
    return results
