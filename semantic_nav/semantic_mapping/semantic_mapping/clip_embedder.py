"""CLIP image/text embedder (open_clip), the real backend behind ``embedder: clip``.

Places crops and free-text queries in one shared vector space so the Phase-2 query server can
match a natural-language query against the image embeddings stored in Phase 1 (this is what the
``MockEmbedder`` only *pretends* to do — its image and text spaces are not actually aligned).

``open_clip``/``torch`` import lazily on first use, so the package imports and the mock pipeline
+ unit tests run with no model download or GPU. The torch-touching feature extraction lives in
``_image_features`` / ``_text_features``; ``embed_image`` / ``embed_text`` only do the pure
BGR->RGB conversion and L2 normalization (both unit-tested by stubbing the extractors).

Dimensionality is fixed by the CLIP model (ViT-B-32 -> 512); the nodes' ``embedding_dim`` param
is ignored when ``embedder: clip``. Use the SAME ``clip_model`` on the mapping node (embed_image)
and the map server (embed_text) so the two spaces line up.
"""
from __future__ import annotations

import numpy as np

from semantic_mapping.enrichment import Embedder


def _to_numpy(arr) -> np.ndarray:
    """Tensor (possibly on GPU) or array-like -> CPU numpy array."""
    if hasattr(arr, "cpu"):          # torch.Tensor
        arr = arr.cpu().numpy()
    return np.asarray(arr)


def _normalize(vec) -> np.ndarray:
    """Flatten to 1-D float32 and L2-normalize (zero vector left as-is)."""
    v = _to_numpy(vec).astype(np.float32).reshape(-1)
    n = float(np.linalg.norm(v))
    return v / n if n > 0.0 else v


class ClipEmbedder(Embedder):
    """open_clip-backed embedder shared across crops (Phase 1) and queries (Phase 2)."""

    def __init__(self, model: str = "ViT-B-32", pretrained: str = "laion2b_s34b_b79k",
                 components=None):
        self.model_name = model
        self.pretrained = pretrained
        # (clip_model, preprocess, tokenizer, device) — lazily populated on first use.
        self._components = components

    def _ensure_loaded(self):
        if self._components is None:
            import open_clip
            import torch
            device = "cuda" if torch.cuda.is_available() else "cpu"
            clip_model, _, preprocess = open_clip.create_model_and_transforms(
                self.model_name, pretrained=self.pretrained, device=device)
            clip_model.eval()
            tokenizer = open_clip.get_tokenizer(self.model_name)
            self._components = (clip_model, preprocess, tokenizer, device)
        return self._components

    def _image_features(self, rgb: np.ndarray) -> np.ndarray:
        import torch
        from PIL import Image
        clip_model, preprocess, _, device = self._ensure_loaded()
        tensor = preprocess(Image.fromarray(rgb)).unsqueeze(0).to(device)
        with torch.no_grad():
            feat = clip_model.encode_image(tensor)
        return _to_numpy(feat)[0]

    def _text_features(self, text: str) -> np.ndarray:
        import torch
        clip_model, _, tokenizer, device = self._ensure_loaded()
        tokens = tokenizer([text]).to(device)
        with torch.no_grad():
            feat = clip_model.encode_text(tokens)
        return _to_numpy(feat)[0]

    # --- Embedder interface ----------------------------------------------

    def embed_image(self, crop_bgr: np.ndarray) -> np.ndarray:
        rgb = np.ascontiguousarray(crop_bgr[..., ::-1])   # BGR -> RGB
        return _normalize(self._image_features(rgb))

    def embed_text(self, text: str) -> np.ndarray:
        return _normalize(self._text_features(text))
