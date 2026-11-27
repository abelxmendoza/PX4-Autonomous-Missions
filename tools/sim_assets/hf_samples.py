"""Fetch free projectsim sample assets (CC-BY-4.0) from Hugging Face.

These are real ProjectSim procedural outputs (USD + manifest), published by
Kaedim. They cost no credits and let the USD -> Gazebo path be tested
before any credits are spent. Revisions are pinned so a re-fetch gets the same
bytes; the sha256 of every file is recorded.

Attribution required by CC-BY-4.0: "projectsim by Kaedim", plus the
dataset URL, which is kept in the provenance written next to the files.
"""
from __future__ import annotations

import json
import zipfile
from pathlib import Path

import httpx

from .gz_model import sha256

# Pinned 2026-10-21 (dataset lastModified 2026-09-30).
REVISIONS = {
    "articulated-logistics-cages": "0b0dc223228ca51b92dad7ca8aa2db16c5d15b66",
    "articulated-boxes": "bbfcf129f30a9c19f4e17c3c27bf1460d0346b23",
    "articulated-desks": "6f0cd4c73515c03b790a1d1bff888d1640fb8da6",
    "articulated-wardrobes": "9a94693bc7964cbf2544865df155e7f2a5ac026f",
}
# dataset -> top-level folder holding <asset>/ directories
FOLDERS = {"articulated-logistics-cages": "cages", "articulated-boxes": "boxes",
           "articulated-desks": "desks", "articulated-wardrobes": "wardrobes"}
MAX_UNZIPPED = 500 * 1024 * 1024


def _url(dataset, path):
    return (f"https://huggingface.co/datasets/projectsim/{dataset}/resolve/"
            f"{REVISIONS[dataset]}/{path}")


def safe_extract(zip_path, dest):
    """Extract refusing absolute paths, '..' and zip bombs."""
    dest = Path(dest).resolve()
    with zipfile.ZipFile(zip_path) as z:
        if sum(i.file_size for i in z.infolist()) > MAX_UNZIPPED:
            raise ValueError(f"{zip_path}: unzipped size exceeds {MAX_UNZIPPED} bytes")
        for info in z.infolist():
            target = (dest / info.filename).resolve()
            if not target.is_relative_to(dest):
                raise ValueError(f"{zip_path}: unsafe member path {info.filename}")
        z.extractall(dest)


def fetch(dataset, asset, root, transport=None):
    """Download, verify and unpack one sample; return the USD path."""
    if dataset not in REVISIONS:
        raise ValueError(f"unknown dataset {dataset}; known: {sorted(REVISIONS)}")
    if not asset.replace("_", "").isalnum():
        raise ValueError("asset id must be [A-Za-z0-9_]")
    dest = Path(root) / dataset / asset
    dest.mkdir(parents=True, exist_ok=True)
    base = f"{FOLDERS[dataset]}/{asset}"
    hashes = {}
    with httpx.Client(transport=transport, follow_redirects=True, timeout=120.0) as http:
        for name in (f"{asset}_usd.zip", "manifest.json"):
            r = http.get(_url(dataset, f"{base}/{name}"))
            r.raise_for_status()
            (dest / name).write_bytes(r.content)
            hashes[name] = sha256(dest / name)
    safe_extract(dest / f"{asset}_usd.zip", dest / "usd")
    usd = dest / "usd" / asset / f"{asset}.usda"
    if not usd.is_file():
        raise ValueError(f"expected {usd} inside the archive")
    provenance = {"source": f"https://huggingface.co/datasets/projectsim/{dataset}",
                  "revision": REVISIONS[dataset], "asset": asset, "license": "CC-BY-4.0",
                  "attribution": "projectsim by Kaedim", "sha256": hashes,
                  "sample_manifest": json.loads((dest / "manifest.json").read_text())}
    (dest / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    return usd
