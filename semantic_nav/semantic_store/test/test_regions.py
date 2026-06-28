"""Region clustering: group spatially co-located entities into zones/rooms."""
from semantic_store import Observation, SpatialSemanticStore
from semantic_store.regions import assign_regions


def test_assign_regions_clusters_nearby_entities():
    s = SpatialSemanticStore(assoc_radius=0.3)
    s.add_observation(Observation("chair", (0.0, 0.0, 0.0)))   # cluster A
    s.add_observation(Observation("table", (1.0, 0.0, 0.0)))
    s.add_observation(Observation("bed", (20.0, 20.0, 0.0)))   # cluster B
    s.add_observation(Observation("lamp", (21.0, 20.0, 0.0)))

    assign_regions(s.entities, eps=2.0)

    region = {e.label: e.region for e in s.entities}
    assert region["chair"] == region["table"]
    assert region["bed"] == region["lamp"]
    assert region["chair"] != region["bed"]
    assert all(e.region for e in s.entities)  # every entity is assigned a region


def test_assign_regions_links_transitively():
    s = SpatialSemanticStore(assoc_radius=0.3)
    # a chain a-b-c where neighbours are within eps but the ends are not
    s.add_observation(Observation("a", (0.0, 0.0, 0.0)))
    s.add_observation(Observation("b", (1.5, 0.0, 0.0)))
    s.add_observation(Observation("c", (3.0, 0.0, 0.0)))

    assign_regions(s.entities, eps=2.0)

    regions = {e.region for e in s.entities}
    assert len(regions) == 1  # transitive linkage merges the whole chain
