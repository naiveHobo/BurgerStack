"""Consolidation: turning a stream of raw observations into stable entities."""
import numpy as np

from semantic_store import Observation, SpatialSemanticStore


def test_first_observation_creates_entity():
    store = SpatialSemanticStore()
    e = store.add_observation(
        Observation("chair", (1.0, 2.0, 0.5), confidence=0.9, stamp=10.0))
    assert len(store.entities) == 1
    assert e.label == "chair"
    assert e.observation_count == 1
    np.testing.assert_allclose(e.position, [1.0, 2.0, 0.5])
    assert e.confidence == 0.9
    assert e.last_seen == 10.0


def test_nearby_same_label_merges_into_one_entity():
    store = SpatialSemanticStore(assoc_radius=0.5)
    store.add_observation(Observation("chair", (1.0, 2.0, 0.0), confidence=0.5, stamp=1.0))
    e = store.add_observation(Observation("chair", (1.2, 2.0, 0.0), confidence=0.8, stamp=2.0))
    assert len(store.entities) == 1
    assert e.observation_count == 2
    np.testing.assert_allclose(e.position, [1.1, 2.0, 0.0])  # running centroid
    assert e.confidence == 0.8                               # aggregate keeps the max
    assert e.last_seen == 2.0


def test_far_same_label_creates_two_entities():
    store = SpatialSemanticStore(assoc_radius=0.5)
    store.add_observation(Observation("chair", (0.0, 0.0, 0.0)))
    store.add_observation(Observation("chair", (5.0, 5.0, 0.0)))
    assert len(store.entities) == 2


def test_nearby_different_label_not_merged_without_embeddings():
    store = SpatialSemanticStore(assoc_radius=0.5)
    store.add_observation(Observation("chair", (0.0, 0.0, 0.0)))
    store.add_observation(Observation("table", (0.1, 0.0, 0.0)))
    assert len(store.entities) == 2


def test_weighted_centroid_after_multiple_merges():
    store = SpatialSemanticStore(assoc_radius=5.0)
    store.add_observation(Observation("box", (0.0, 0.0, 0.0)))
    store.add_observation(Observation("box", (0.0, 0.0, 0.0)))
    e = store.add_observation(Observation("box", (3.0, 0.0, 0.0)))
    assert e.observation_count == 3
    np.testing.assert_allclose(e.position, [1.0, 0.0, 0.0])  # (0 + 0 + 3) / 3


def test_embedding_similarity_merges_synonyms_and_records_alias():
    store = SpatialSemanticStore(assoc_radius=0.5, embedding_sim_threshold=0.9)
    a = np.array([1.0, 0.0, 0.0])
    b = np.array([0.99, 0.01, 0.0])  # cosine with a is ~1.0
    store.add_observation(Observation("sofa", (0.0, 0.0, 0.0), embedding=a))
    e = store.add_observation(Observation("couch", (0.2, 0.0, 0.0), embedding=b))
    assert len(store.entities) == 1
    assert e.observation_count == 2
    assert "couch" in e.aliases  # canonical label stays "sofa", synonym recorded
