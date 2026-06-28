"""Open-vocabulary (YOLO-World) detector: parsing ultralytics Results into Detection2D.

The model (ultralytics/torch) is never imported here. The detector lazy-loads it, and the
parse logic is exercised with a fake Results object, mirroring the package's mock-first design
(heavy ML imports happen only when a real detector is actually selected at runtime).
"""
import numpy as np
import pytest

from semantic_perception.detector import Detection2D
from semantic_perception.openvocab_detector import OpenVocabDetector, _parse_results


class _FakeBoxes:
    """Mimics ultralytics Results.boxes (xyxy / conf / cls as array-likes)."""

    def __init__(self, xyxy, conf, cls):
        self.xyxy = np.asarray(xyxy, dtype=np.float32).reshape(-1, 4)
        self.conf = np.asarray(conf, dtype=np.float32).reshape(-1)
        self.cls = np.asarray(cls, dtype=np.float32).reshape(-1)


class _FakeResults:
    def __init__(self, boxes):
        self.boxes = boxes


VOCAB = ["chair", "table", "person"]


def test_parse_results_maps_boxes_to_detections():
    boxes = _FakeBoxes(
        xyxy=[[10, 20, 50, 80], [100, 100, 140, 160]],
        conf=[0.9, 0.5],
        cls=[0, 2])
    dets = _parse_results([_FakeResults(boxes)], VOCAB)
    assert len(dets) == 2
    d0 = dets[0]
    assert isinstance(d0, Detection2D)
    assert d0.label == "chair"
    assert d0.score == pytest.approx(0.9)
    # xyxy -> (x, y, w, h)
    assert (d0.x, d0.y, d0.width, d0.height) == (10, 20, 40, 60)
    assert dets[1].label == "person"
    assert (dets[1].x, dets[1].y, dets[1].width, dets[1].height) == (100, 100, 40, 60)


def test_parse_results_emits_python_scalars():
    boxes = _FakeBoxes(xyxy=[[1, 2, 3, 4]], conf=[0.7], cls=[1])
    d = _parse_results([_FakeResults(boxes)], VOCAB)[0]
    assert type(d.x) is int and type(d.width) is int
    assert type(d.score) is float
    assert d.label == "table"


def test_parse_results_empty_when_no_boxes():
    boxes = _FakeBoxes(xyxy=np.zeros((0, 4)), conf=[], cls=[])
    assert _parse_results([_FakeResults(boxes)], VOCAB) == []


def test_parse_results_handles_empty_and_none():
    assert _parse_results([], VOCAB) == []
    assert _parse_results([_FakeResults(None)], VOCAB) == []


def test_parse_results_unknown_class_index_falls_back_to_str():
    boxes = _FakeBoxes(xyxy=[[0, 0, 10, 10]], conf=[1.0], cls=[7])
    d = _parse_results([_FakeResults(boxes)], VOCAB)[0]
    assert d.label == "7"


def test_detect_uses_injected_model_and_threshold():
    boxes = _FakeBoxes(xyxy=[[0, 0, 20, 20]], conf=[0.8], cls=[0])

    class _FakeModel:
        def __init__(self):
            self.classes = None
            self.last_conf = None

        def set_classes(self, vocab):
            self.classes = list(vocab)

        def predict(self, img, conf=0.0, verbose=False):
            self.last_conf = conf
            return [_FakeResults(boxes)]

    fake = _FakeModel()
    det = OpenVocabDetector(vocabulary=VOCAB, confidence_threshold=0.3, model=fake)
    # vocabulary is pushed into the model on construction
    assert fake.classes == VOCAB
    out = det.detect(np.zeros((64, 64, 3), dtype=np.uint8))
    assert len(out) == 1 and out[0].label == "chair"
    # the confidence threshold is forwarded to predict()
    assert fake.last_conf == pytest.approx(0.3)
