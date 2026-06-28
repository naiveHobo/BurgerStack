"""VLM crop describer over a local ollama server, the real backend behind ``describer: ollama``.

Sends each retained crop to a small vision-language model (default ``moondream``) and stores
its short natural-language description on the entity (vs. ``MockDescriber``'s colour statistics).
These descriptions feed Phase-2 retrieval and make the map human-readable.

The ``ollama`` client imports lazily and is injectable, so the package imports and the unit
tests run with no server, model pull, or network. The PNG encoding and response parsing are
pure module-level helpers (``_encode_png`` / ``_extract_text``) and unit-tested directly.
"""
from __future__ import annotations

import cv2
import numpy as np

from semantic_mapping.enrichment import Describer

DEFAULT_PROMPT = (
    "Describe this object in a short phrase (a few words): its kind, colour, and any "
    "distinctive feature. Reply with the phrase only.")


def _encode_png(crop_bgr: np.ndarray) -> bytes:
    """Encode a BGR crop as PNG bytes (lossless; cv2 writes correct RGB colours)."""
    ok, buf = cv2.imencode(".png", crop_bgr)
    if not ok:
        raise ValueError("failed to PNG-encode crop")
    return buf.tobytes()


def _extract_text(resp) -> str:
    """Pull the assistant text out of an ollama ChatResponse (object or dict form)."""
    msg = getattr(resp, "message", None)
    if msg is None and isinstance(resp, dict):
        msg = resp.get("message")
    content = getattr(msg, "content", None)
    if content is None and isinstance(msg, dict):
        content = msg.get("content")
    return content or ""


class OllamaDescriber(Describer):
    """Describes crops with a local ollama vision model."""

    def __init__(self, model: str = "moondream", host: str = "http://localhost:11434",
                 client=None, prompt: str = DEFAULT_PROMPT):
        self.model = model
        self.host = host
        self.prompt = prompt
        self._client = client

    @property
    def client(self):
        if self._client is None:
            import ollama  # lazy: only when a real describer is selected
            self._client = ollama.Client(host=self.host)
        return self._client

    def describe(self, crop_bgr: np.ndarray) -> str:
        png = _encode_png(crop_bgr)
        resp = self.client.chat(
            model=self.model,
            messages=[{"role": "user", "content": self.prompt, "images": [png]}])
        return _extract_text(resp).strip()
