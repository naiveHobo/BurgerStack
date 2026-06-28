"""The spatial semantic store: accumulates observations into consolidated entities."""
from __future__ import annotations

import json
from pathlib import Path
from typing import List, Optional

import numpy as np

from . import association, retrieval
from .schema import Observation, SemanticEntity, as_vec3

SCHEMA_VERSION = 1


class SpatialSemanticStore:
    """In-memory spatial semantic memory.

    Observations are associated into :class:`SemanticEntity` objects (see
    :mod:`semantic_store.association`); querying and persistence live in the
    ``retrieval`` and (de)serialization helpers.
    """

    def __init__(self, *, assoc_radius: float = 0.5, embedding_sim_threshold: float = 0.85):
        self.assoc_radius = assoc_radius
        self.embedding_sim_threshold = embedding_sim_threshold
        self._entities: List[SemanticEntity] = []
        self._next_id = 0

    @property
    def entities(self) -> List[SemanticEntity]:
        return self._entities

    def add_observation(self, obs: Observation) -> SemanticEntity:
        """Fuse ``obs`` into the nearest compatible entity, or create a new one."""
        best: Optional[SemanticEntity] = None
        best_dist = float("inf")
        for e in self._entities:
            if association.can_associate(
                e, obs,
                assoc_radius=self.assoc_radius,
                embedding_sim_threshold=self.embedding_sim_threshold,
            ):
                d = float(np.linalg.norm(e.position - obs.position))
                if d < best_dist:
                    best, best_dist = e, d

        if best is not None:
            association.merge_observation(best, obs)
            return best

        entity = SemanticEntity(
            id=self._next_id,
            label=obs.label,
            position=obs.position.copy(),
            confidence=obs.confidence,
            observation_count=1,
            last_seen=obs.stamp,
            embedding=None if obs.embedding is None else obs.embedding.copy(),
            size=None if obs.size is None else obs.size.copy(),
        )
        self._next_id += 1
        self._entities.append(entity)
        return entity

    def query(
        self,
        *,
        text_embedding=None,
        label: Optional[str] = None,
        region: Optional[str] = None,
        near=None,
        radius: Optional[float] = None,
        top_k: Optional[int] = None,
    ) -> List[SemanticEntity]:
        """Ranked retrieval over the stored entities (see :mod:`semantic_store.retrieval`)."""
        return retrieval.query(
            self._entities,
            text_embedding=text_embedding,
            label=label,
            region=region,
            near=near,
            radius=radius,
            top_k=top_k,
        )

    # --- persistence -----------------------------------------------------

    @staticmethod
    def _entity_to_dict(e: SemanticEntity) -> dict:
        return {
            "id": e.id,
            "label": e.label,
            "aliases": list(e.aliases),
            "description": e.description,
            "region": e.region,
            "position": e.position.tolist(),
            "size": None if e.size is None else e.size.tolist(),
            "confidence": e.confidence,
            "observation_count": e.observation_count,
            "last_seen": e.last_seen,
            "embedding": None if e.embedding is None else e.embedding.tolist(),
        }

    def to_dict(self) -> dict:
        return {
            "version": SCHEMA_VERSION,
            "assoc_radius": self.assoc_radius,
            "embedding_sim_threshold": self.embedding_sim_threshold,
            "entities": [self._entity_to_dict(e) for e in self._entities],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "SpatialSemanticStore":
        store = cls(
            assoc_radius=d.get("assoc_radius", 0.5),
            embedding_sim_threshold=d.get("embedding_sim_threshold", 0.85),
        )
        for ed in d.get("entities", []):
            emb = ed.get("embedding")
            size = ed.get("size")
            store._entities.append(SemanticEntity(
                id=ed["id"],
                label=ed["label"],
                position=as_vec3(ed["position"]),
                confidence=ed.get("confidence", 0.0),
                observation_count=ed.get("observation_count", 0),
                last_seen=ed.get("last_seen", 0.0),
                aliases=list(ed.get("aliases", [])),
                description=ed.get("description", ""),
                region=ed.get("region", ""),
                embedding=None if emb is None else np.asarray(emb, dtype=float),
                size=None if size is None else as_vec3(size),
            ))
        store._next_id = max((e.id for e in store._entities), default=-1) + 1
        return store

    def save(self, path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load(cls, path) -> "SpatialSemanticStore":
        with open(Path(path)) as f:
            return cls.from_dict(json.load(f))
