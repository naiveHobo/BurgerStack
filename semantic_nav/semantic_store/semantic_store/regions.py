"""Spatial clustering of entities into regions (zones / rooms).

Single-linkage connected components over horizontal (x, y) proximity: entities within
``eps`` of one another land in the same region, transitively. This assigns generic
region ids (``region_0`` ...); human-readable names (e.g. "kitchen") are layered on
top elsewhere using the VLM / map context.
"""
from __future__ import annotations

from typing import Sequence

import numpy as np

from .schema import SemanticEntity


def assign_regions(entities: Sequence[SemanticEntity], *, eps: float, prefix: str = "region_") -> None:
    """Assign ``entity.region`` in place by clustering positions within ``eps`` (metres)."""
    n = len(entities)
    parent = list(range(n))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]  # path compression
            i = parent[i]
        return i

    def union(i: int, j: int) -> None:
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[ri] = rj

    pos = [e.position[:2] for e in entities]  # cluster in the floor plane
    for i in range(n):
        for j in range(i + 1, n):
            if float(np.linalg.norm(pos[i] - pos[j])) <= eps:
                union(i, j)

    # Deterministic numbering by first appearance of each cluster root.
    label_of: dict[int, str] = {}
    for i in range(n):
        root = find(i)
        if root not in label_of:
            label_of[root] = f"{prefix}{len(label_of)}"
        entities[i].region = label_of[root]
