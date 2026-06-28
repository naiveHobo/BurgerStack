"""CLIP embedder: BGR->RGB handling + L2 normalization.

The torch/open_clip feature extraction is stubbed (subclass overrides ``_image_features`` /
``_text_features``), so no model download or GPU is needed at test time. This isolates the
contract that actually matters for retrieval: crops go in as BGR, embeddings come out unit-norm
and float32, and image/text land in the same-dimensioned space.
"""
import numpy as np

from semantic_mapping.clip_embedder import ClipEmbedder, _normalize


def test_normalize_returns_unit_float32_1d():
    out = _normalize(np.array([3.0, 4.0]))   # norm 5
    assert out.dtype == np.float32
    assert out.shape == (2,)
    np.testing.assert_allclose(np.linalg.norm(out), 1.0, atol=1e-6)
    np.testing.assert_allclose(out, [0.6, 0.8], atol=1e-6)


def test_normalize_handles_zero_vector():
    out = _normalize(np.zeros(4))
    assert out.shape == (4,) and np.all(out == 0)


class _StubEmbedder(ClipEmbedder):
    """Records what reaches the feature extractors; returns fixed non-unit vectors."""

    def __init__(self):
        super().__init__()
        self.seen_image = None
        self.seen_text = None

    def _image_features(self, rgb):
        self.seen_image = rgb
        return np.array([0.0, 3.0, 4.0])      # length 5, not unit

    def _text_features(self, text):
        self.seen_text = text
        return np.array([4.0, 0.0, 3.0])


def test_embed_image_converts_bgr_to_rgb_and_normalizes():
    emb = _StubEmbedder()
    crop = np.zeros((2, 2, 3), dtype=np.uint8)
    crop[..., 0] = 10   # B
    crop[..., 1] = 20   # G
    crop[..., 2] = 30   # R
    out = emb.embed_image(crop)
    # the extractor must have received RGB: channel 0 was R (30), channel 2 was B (10)
    assert emb.seen_image[0, 0, 0] == 30
    assert emb.seen_image[0, 0, 2] == 10
    np.testing.assert_allclose(np.linalg.norm(out), 1.0, atol=1e-6)
    assert out.dtype == np.float32


def test_embed_text_normalizes_and_passes_text_through():
    emb = _StubEmbedder()
    out = emb.embed_text("a red chair")
    assert emb.seen_text == "a red chair"
    np.testing.assert_allclose(np.linalg.norm(out), 1.0, atol=1e-6)
    assert out.dtype == np.float32


def test_image_and_text_share_dimensionality():
    emb = _StubEmbedder()
    vi = emb.embed_image(np.zeros((2, 2, 3), dtype=np.uint8))
    vt = emb.embed_text("x")
    assert vi.shape == vt.shape
