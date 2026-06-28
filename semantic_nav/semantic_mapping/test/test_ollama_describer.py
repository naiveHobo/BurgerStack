"""Ollama VLM describer (moondream): PNG encoding + response extraction.

The ollama client is injected (a fake), so no server/model/network is needed at test time —
only the pure PNG round-trip and the response-parsing are exercised.
"""
from types import SimpleNamespace

import cv2
import numpy as np

from semantic_mapping.ollama_describer import OllamaDescriber, _encode_png, _extract_text


def test_encode_png_roundtrips_to_same_bgr_array():
    crop = np.zeros((6, 8, 3), dtype=np.uint8)
    crop[..., 0], crop[..., 1], crop[..., 2] = 10, 20, 30
    png = _encode_png(crop)
    assert isinstance(png, (bytes, bytearray))
    decoded = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (6, 8, 3)
    np.testing.assert_array_equal(decoded, crop)   # PNG is lossless; colours preserved


def test_extract_text_from_object_and_dict():
    obj = SimpleNamespace(message=SimpleNamespace(content="a wooden chair"))
    assert _extract_text(obj) == "a wooden chair"
    assert _extract_text({"message": {"content": "a table"}}) == "a table"
    assert _extract_text({}) == ""


class _FakeClient:
    def __init__(self, text):
        self.text = text
        self.calls = []

    def chat(self, model, messages):
        self.calls.append((model, messages))
        return SimpleNamespace(message=SimpleNamespace(content=self.text))


def test_describe_sends_image_and_returns_stripped_text():
    fake = _FakeClient("  a red mug  ")
    d = OllamaDescriber(model="moondream", client=fake)
    out = d.describe(np.full((4, 4, 3), 100, dtype=np.uint8))
    assert out == "a red mug"

    model, messages = fake.calls[0]
    assert model == "moondream"
    msg = messages[0]
    assert msg["role"] == "user" and msg["content"]
    assert len(msg["images"]) == 1
    assert isinstance(msg["images"][0], (bytes, bytearray))
