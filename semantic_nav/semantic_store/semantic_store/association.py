"""Deciding when two observations describe the same physical object, and fusing them.

This is the logic that turns a noisy stream of detections into stable entities — and
the reason a re-observed object is *updated* rather than duplicated (the failure mode
called out as a limitation in the reference design).
"""
from __future__ import annotations

import numpy as np

from .schema import Observation, SemanticEntity


def cosine_similarity(a: np.ndarray, b: np.ndarray) -> float:
    na = float(np.linalg.norm(a))
    nb = float(np.linalg.norm(b))
    if na == 0.0 or nb == 0.0:
        return 0.0
    return float(np.dot(a, b) / (na * nb))


def can_associate(
    entity: SemanticEntity,
    obs: Observation,
    *,
    assoc_radius: float,
    embedding_sim_threshold: float,
) -> bool:
    """An observation joins an entity only if it is both spatially close AND
    semantically compatible (same label/alias, or similar enough embedding)."""
    if float(np.linalg.norm(entity.position - obs.position)) > assoc_radius:
        return False

    obs_label = obs.label.lower()
    if obs_label == entity.label.lower():
        return True
    if obs_label in (a.lower() for a in entity.aliases):
        return True
    if entity.embedding is not None and obs.embedding is not None:
        if cosine_similarity(entity.embedding, obs.embedding) >= embedding_sim_threshold:
            return True
    return False


def merge_observation(entity: SemanticEntity, obs: Observation) -> None:
    """Fold ``obs`` into ``entity`` in place: running centroid + aggregates."""
    n = entity.observation_count
    new_n = n + 1

    entity.position = (entity.position * n + obs.position) / new_n
    entity.observation_count = new_n
    entity.confidence = max(entity.confidence, obs.confidence)
    entity.last_seen = max(entity.last_seen, obs.stamp)

    if obs.label.lower() != entity.label.lower() and obs.label not in entity.aliases:
        entity.aliases.append(obs.label)

    if obs.embedding is not None:
        entity.embedding = (
            obs.embedding.copy()
            if entity.embedding is None
            else (entity.embedding * n + obs.embedding) / new_n
        )

    if obs.size is not None:
        entity.size = (
            obs.size.copy() if entity.size is None else np.maximum(entity.size, obs.size)
        )
