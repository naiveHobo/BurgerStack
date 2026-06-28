"""Batch enrichment backends + the enrich_store orchestration. Pure (no ROS)."""
import numpy as np

from semantic_store.schema import Observation
from semantic_store.store import SpatialSemanticStore

from semantic_mapping.enrichment import (
    Describer,
    Embedder,
    MockDescriber,
    MockEmbedder,
    enrich_store,
)

CROP_A = np.full((4, 4, 3), 30, dtype=np.uint8)
CROP_B = np.full((4, 4, 3), 200, dtype=np.uint8)


# --- MockEmbedder --------------------------------------------------------

def test_embed_image_is_deterministic():
    emb = MockEmbedder(dim=8)
    np.testing.assert_array_equal(emb.embed_image(CROP_A), emb.embed_image(CROP_A))


def test_embed_image_has_requested_dim_and_is_normalized():
    v = MockEmbedder(dim=16).embed_image(CROP_A)
    assert v.shape == (16,)
    np.testing.assert_allclose(np.linalg.norm(v), 1.0, atol=1e-6)


def test_embed_image_differs_by_content():
    emb = MockEmbedder(dim=8)
    assert not np.allclose(emb.embed_image(CROP_A), emb.embed_image(CROP_B))


def test_embed_text_is_deterministic_and_normalized():
    emb = MockEmbedder(dim=8)
    np.testing.assert_array_equal(emb.embed_text("chair"), emb.embed_text("chair"))
    np.testing.assert_allclose(np.linalg.norm(emb.embed_text("chair")), 1.0, atol=1e-6)


def test_embed_text_differs_by_content():
    emb = MockEmbedder(dim=8)
    assert not np.allclose(emb.embed_text("chair"), emb.embed_text("table"))


# --- MockDescriber -------------------------------------------------------

def test_describe_is_deterministic_and_nonempty():
    d = MockDescriber()
    assert d.describe(CROP_A) == d.describe(CROP_A)
    assert isinstance(d.describe(CROP_A), str)
    assert d.describe(CROP_A)


# --- enrich_store --------------------------------------------------------

def _store_with_two_entities():
    store = SpatialSemanticStore(assoc_radius=0.5)
    e0 = store.add_observation(Observation(label="chair", position=[0.0, 0.0, 0.0]))
    e1 = store.add_observation(Observation(label="table", position=[5.0, 5.0, 0.0]))
    return store, e0.id, e1.id


def test_enrich_store_fills_only_entities_with_crops():
    store, id0, id1 = _store_with_two_entities()
    n = enrich_store(
        store, {id0: CROP_A},
        describer=MockDescriber(), embedder=MockEmbedder(dim=8), region_eps=0.0)

    by_id = {e.id: e for e in store.entities}
    assert n == 1
    assert by_id[id0].description           # filled
    assert by_id[id0].embedding is not None
    assert by_id[id0].embedding.shape == (8,)
    assert by_id[id1].description == ""      # no crop -> untouched
    assert by_id[id1].embedding is None


def test_enrich_store_assigns_regions_when_eps_positive():
    store, id0, id1 = _store_with_two_entities()
    enrich_store(store, {}, region_eps=1.0)
    # Two entities 7 m apart -> two distinct regions, both non-empty.
    regions = {e.region for e in store.entities}
    assert "" not in regions
    assert len(regions) == 2


def test_enrich_store_skips_regions_when_eps_zero():
    store, id0, id1 = _store_with_two_entities()
    enrich_store(store, {}, region_eps=0.0)
    assert all(e.region == "" for e in store.entities)


def test_enrich_store_without_backends_only_clusters():
    store, id0, id1 = _store_with_two_entities()
    n = enrich_store(store, {id0: CROP_A}, region_eps=1.0)
    assert n == 0  # no describer/embedder -> nothing enriched
    assert all(e.description == "" and e.embedding is None for e in store.entities)


# --- enrich_store resilience (one bad crop must not abort the batch) ------

class _CropSelectiveDescriber(Describer):
    """Describes the dark crop, but raises on the bright one."""

    def describe(self, crop_bgr):
        if crop_bgr.mean() > 100:  # CROP_B
            raise RuntimeError("describe boom")
        return "ok"


class _CropSelectiveEmbedder(Embedder):
    """Embeds the dark crop, but raises on the bright one."""

    def embed_image(self, crop_bgr):
        if crop_bgr.mean() > 100:  # CROP_B
            raise RuntimeError("embed boom")
        return np.ones(8, dtype=np.float32) / np.sqrt(8.0)

    def embed_text(self, text):  # pragma: no cover - unused here
        return np.ones(8, dtype=np.float32) / np.sqrt(8.0)


def test_enrich_store_does_not_propagate_backend_errors():
    store, id0, id1 = _store_with_two_entities()
    errors = []

    n = enrich_store(
        store, {id0: CROP_A, id1: CROP_B},
        describer=_CropSelectiveDescriber(), embedder=_CropSelectiveEmbedder(),
        region_eps=0.0,
        on_error=lambda eid, stage, exc: errors.append((eid, stage)))

    by_id = {e.id: e for e in store.entities}
    # Good crop fully enriched; bad crop skipped, not crashed.
    assert n == 1
    assert by_id[id0].description == "ok"
    assert by_id[id0].embedding is not None
    assert by_id[id1].description == ""       # backend raised -> left untouched
    assert by_id[id1].embedding is None
    # The failure was reported per-entity per-stage, not swallowed silently.
    assert (id1, "describe") in errors
    assert (id1, "embed") in errors
