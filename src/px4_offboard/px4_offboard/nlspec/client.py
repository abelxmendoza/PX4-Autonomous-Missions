"""Minimal OpenAI-compatible chat client (standard library only).

Works with llama.cpp `llama-server`, Ollama (`/v1`), vLLM, OpenRouter, or any
OpenAI-compatible endpoint. Configure with arguments or environment:

    NLSPEC_BASE_URL  default http://localhost:8080/v1  (llama-server's default port)
    NLSPEC_MODEL     default "local"
    NLSPEC_API_KEY   optional; sent as a Bearer token when set
"""

from __future__ import annotations

import json
import os
import time
import urllib.error
import urllib.request
from dataclasses import dataclass


@dataclass(frozen=True)
class ChatResult:
    text: str
    latency_s: float
    model: str
    prompt_tokens: int | None = None
    completion_tokens: int | None = None

    @property
    def completion_tps(self) -> float | None:
        if not self.completion_tokens or self.latency_s <= 0:
            return None
        return self.completion_tokens / self.latency_s


class ChatError(RuntimeError):
    pass


@dataclass
class OpenAICompatClient:
    base_url: str = ""
    model: str = ""
    api_key: str | None = None
    temperature: float = 0.0
    max_tokens: int = 1200
    timeout_s: float = 300.0

    def __post_init__(self) -> None:
        self.base_url = (self.base_url or os.environ.get("NLSPEC_BASE_URL", "http://localhost:8080/v1")).rstrip("/")
        self.model = self.model or os.environ.get("NLSPEC_MODEL", "local")
        if self.api_key is None:
            self.api_key = os.environ.get("NLSPEC_API_KEY") or None

    def __call__(self, messages: list[dict]) -> ChatResult:
        body = json.dumps({
            "model": self.model,
            "messages": messages,
            "temperature": self.temperature,
            "max_tokens": self.max_tokens,
        }).encode()
        headers = {"Content-Type": "application/json"}
        if self.api_key:
            headers["Authorization"] = f"Bearer {self.api_key}"
        req = urllib.request.Request(f"{self.base_url}/chat/completions", data=body, headers=headers)
        start = time.monotonic()
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                payload = json.loads(resp.read())
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
            raise ChatError(f"chat request to {self.base_url} failed: {exc}") from exc
        latency = time.monotonic() - start
        try:
            text = payload["choices"][0]["message"]["content"] or ""
        except (KeyError, IndexError, TypeError) as exc:
            raise ChatError(f"unexpected response shape: {str(payload)[:300]}") from exc
        usage = payload.get("usage") or {}
        return ChatResult(
            text=text,
            latency_s=latency,
            model=payload.get("model") or self.model,
            prompt_tokens=usage.get("prompt_tokens"),
            completion_tokens=usage.get("completion_tokens"),
        )
