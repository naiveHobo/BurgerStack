"""Detector abstraction + the deterministic MockDetector used to validate the pipeline."""
import numpy as np
import pytest

from semantic_perception.detector import Detection2D, Detector, MockDetector


def _img(h=480, w=640):
    return np.zeros((h, w, 3), dtype=np.uint8)


def test_detector_is_abstract():
    with pytest.raises(TypeError):
        Detector()  # cannot instantiate an abstract base


def test_mock_detector_returns_single_labelled_detection():
    dets = MockDetector(label="chair").detect(_img())
    assert len(dets) == 1
    d = dets[0]
    assert isinstance(d, Detection2D)
    assert d.label == "chair"
    assert d.score == 1.0


def test_mock_detector_box_is_centered_and_in_bounds():
    h, w = 480, 640
    d = MockDetector(box_frac=0.33).detect(_img(h, w))[0]
    assert 0 <= d.x and 0 <= d.y
    assert d.x + d.width <= w and d.y + d.height <= h
    # bbox centre coincides with the image centre (within a pixel)
    assert abs((d.x + d.width / 2) - w / 2) <= 1
    assert abs((d.y + d.height / 2) - h / 2) <= 1


def test_mock_detector_is_deterministic():
    img = _img()
    a = MockDetector().detect(img)[0]
    b = MockDetector().detect(img)[0]
    assert (a.x, a.y, a.width, a.height, a.label, a.score) == \
           (b.x, b.y, b.width, b.height, b.label, b.score)
