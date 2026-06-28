"""JSON persistence: the semantic map is a portable artifact saved next to the slam map."""
import numpy as np

from semantic_store import Observation, SpatialSemanticStore


def test_save_load_roundtrip(tmp_path):
    s = SpatialSemanticStore()
    s.add_observation(Observation("chair", (1.0, 2.0, 0.3), confidence=0.9,
                                  embedding=np.array([0.1, 0.2, 0.3])))
    s.entities[0].description = "a wooden chair"
    s.entities[0].region = "office"

    path = tmp_path / "semantic_map.json"
    s.save(path)
    assert path.exists()

    loaded = SpatialSemanticStore.load(path)
    assert len(loaded.entities) == 1
    e = loaded.entities[0]
    assert e.label == "chair"
    assert e.description == "a wooden chair"
    assert e.region == "office"
    assert e.confidence == 0.9
    np.testing.assert_allclose(e.position, [1.0, 2.0, 0.3])
    np.testing.assert_allclose(e.embedding, [0.1, 0.2, 0.3])


def test_query_survives_roundtrip(tmp_path):
    s = SpatialSemanticStore()
    s.add_observation(Observation("chair", (1.0, 1.0, 0.0)))
    p = tmp_path / "m.json"
    s.save(p)
    assert SpatialSemanticStore.load(p).query(label="chair")[0].label == "chair"


def test_ids_stay_unique_when_adding_after_load(tmp_path):
    s = SpatialSemanticStore(assoc_radius=0.1)
    s.add_observation(Observation("a", (0.0, 0.0, 0.0)))
    s.add_observation(Observation("b", (5.0, 0.0, 0.0)))
    p = tmp_path / "m.json"
    s.save(p)

    loaded = SpatialSemanticStore.load(p)
    loaded.add_observation(Observation("c", (9.0, 0.0, 0.0)))
    ids = [e.id for e in loaded.entities]
    assert len(ids) == len(set(ids))  # _next_id was restored, so no collisions
