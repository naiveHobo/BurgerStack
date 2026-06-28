"""Ranked retrieval: label / region / spatial / embedding queries over the store."""
import numpy as np

from semantic_store import Observation, SpatialSemanticStore


def _store():
    s = SpatialSemanticStore(assoc_radius=0.4)
    s.add_observation(Observation("chair", (1.0, 1.0, 0.0), confidence=0.9))
    s.add_observation(Observation("table", (2.0, 2.0, 0.0), confidence=0.8))
    s.add_observation(Observation("fire extinguisher", (8.0, 0.0, 0.0), confidence=0.7))
    return s


def test_query_by_label_exact():
    res = _store().query(label="chair")
    assert len(res) == 1 and res[0].label == "chair"


def test_query_by_label_is_case_insensitive_and_substring():
    res = _store().query(label="Fire")
    assert len(res) == 1 and res[0].label == "fire extinguisher"


def test_query_near_radius_filters_spatially():
    res = _store().query(near=(1.0, 1.0, 0.0), radius=1.0)
    assert {e.label for e in res} == {"chair"}  # table is ~1.41 m away


def test_query_near_ranks_by_proximity_when_no_radius():
    res = _store().query(near=(2.0, 2.0, 0.0))
    assert res[0].label == "table"  # nearest first


def test_query_top_k_limits_results():
    res = _store().query(top_k=2)
    assert len(res) == 2


def test_query_by_text_embedding_ranks_by_cosine():
    s = SpatialSemanticStore()
    s.add_observation(Observation("ball", (0.0, 0.0, 0.0), embedding=np.array([1.0, 0.0])))
    s.add_observation(Observation("mug", (9.0, 0.0, 0.0), embedding=np.array([0.0, 1.0])))
    res = s.query(text_embedding=np.array([0.9, 0.1]))
    assert res[0].label == "ball"


def test_query_by_region():
    s = _store()
    for e in s.entities:
        e.region = "kitchen" if e.label == "table" else "lobby"
    res = s.query(region="lobby")
    assert {e.label for e in res} == {"chair", "fire extinguisher"}
