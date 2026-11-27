"""OPTIONAL ProjectSim (api.projectsim.ai) mesh source with a spend guard.

Nothing else in sim_assets imports this module; the pipeline runs on local
meshes without it.

Facts used here come from docs.projectsim.ai (read 2026-10-21):
  * POST /v1/assets/standard | /v1/assets/measured -> 202 {id, job_id, ...}
  * POST /v1/variation-sets/procedural (count 10-500, 1 credit each, USD
    only, rigid, optional joints) | /high-fidelity (count 1-100, 4 each)
    -> 202 {id, job_id, count}; members via GET /v1/variation-sets/{id}/assets
  * GET /v1/jobs/{id}: queued -> running -> complete | failed | canceled
  * GET /v1/assets/{id}: metadata + files[{path, url}] (URLs live 1 h)
  * standard / measured / high-fidelity asset = 4 credits, procedural
    member = 1; a failed job is refunded; gift credits are used first
  * physics.simulator picks the output format; there is no SDF/Gazebo
    target. "agnostic" = FBX + GLB + physics sidecar JSON (rigid only),
    which is what this pipeline consumes.
  * standard defaults to articulated generation (Isaac only); reconstruction
    needs articulated: false and collision_granularity: null.

Safety rules enforced in code, not by convention:
  * the token is read from PROJECTSIM_TOKEN or a chmod-600 file outside the
    repo (~/.config/projectsim/token) and never written anywhere
  * nothing that costs credits is sent without confirm_spend=True, and never
    past the plan budget minus its reserve
  * every spend is appended to a local JSONL ledger BEFORE the request, so a
    crash cannot hide a spend; unresolved intents count as spent
  * identical request bodies reuse one Idempotency-Key, so a retry after a
    timeout replays instead of buying the asset twice
  * presigned download URLs are stripped from stored metadata
"""
from __future__ import annotations

import copy
import datetime as dt
import hashlib
import json
import os
from pathlib import Path

import httpx
import yaml

BASE_URL = "https://api.projectsim.ai"
# credits per created asset; sets are charged per member
CREDITS = {"standard": 4, "measured": 4, "procedural": 1, "high_fidelity": 4}
SET_COUNT = {"procedural": (10, 500), "high_fidelity": (1, 100)}
ENDPOINTS = {"standard": "/v1/assets/standard", "measured": "/v1/assets/measured",
             "procedural": "/v1/variation-sets/procedural",
             "high_fidelity": "/v1/variation-sets/high-fidelity"}
TERMINAL = {"complete", "failed", "canceled", "partial"}
COSTS_SOURCE = "https://docs.projectsim.ai/getting-started/authentication.md (2026-10-21)"


class ProjectSimError(RuntimeError):
    def __init__(self, status, problem):
        self.status = status
        self.problem = problem
        super().__init__(f"HTTP {status}: {problem.get('title', '')} - {problem.get('detail', '')} "
                         f"(request_id={problem.get('request_id')})")


class SpendNotConfirmed(RuntimeError):
    pass


class BudgetExceeded(RuntimeError):
    pass


# --------------------------------------------------------------------- plan

def load_plan(path):
    return load_plan_dict(yaml.safe_load(Path(path).read_text(encoding="utf-8")))


def load_plan_dict(plan):
    if not isinstance(plan, dict) or plan.get("format") != "projectsim-plan" or plan.get("version") != 1:
        raise ValueError("expected format: projectsim-plan, version: 1")
    budget = plan.get("budget") or {}
    for key in ("credits", "reserve_credits"):
        if type(budget.get(key)) is not int or budget[key] < 0:
            raise ValueError(f"budget.{key} must be a non-negative integer")
    ids = set()
    for item in plan.get("items") or []:
        if item.get("endpoint") not in CREDITS:
            raise ValueError(f"{item.get('id')}: endpoint must be one of {sorted(CREDITS)}")
        if item["endpoint"] in SET_COUNT:
            lo, hi = SET_COUNT[item["endpoint"]]
            if type(item.get("count")) is not int or not lo <= item["count"] <= hi:
                raise ValueError(f"{item.get('id')}: {item['endpoint']} count must be {lo}-{hi}")
            if not isinstance(item.get("seed"), dict):
                raise ValueError(f"{item.get('id')}: variation sets need a seed")
        elif not isinstance(item.get("inputs"), dict):
            raise ValueError(f"{item.get('id')}: {item['endpoint']} needs inputs")
        if not item.get("id") or item["id"] in ids:
            raise ValueError(f"duplicate or missing item id: {item.get('id')}")
        ids.add(item["id"])
    return plan


def request_body(plan, item):
    """Merge per-endpoint plan defaults with the item; the exact JSON sent.

    Defaults are keyed by endpoint because endpoints reject fields they do
    not support (procedural rejects mesh/texture settings, for example)."""
    defaults = (plan.get("defaults") or {}).get(item["endpoint"]) or {}
    params = copy.deepcopy(defaults.get("params", {}))
    for section, values in (item.get("params") or {}).items():
        params.setdefault(section, {}).update(values)
    body = {"name": item.get("name", item["id"])}
    if item["endpoint"] in SET_COUNT:
        body.update(count=item["count"], seed=item["seed"])
    else:
        body["inputs"] = item["inputs"]
    if params:
        body["params"] = params
    return body


def idempotency_key(endpoint, body):
    canonical = json.dumps([endpoint, body], sort_keys=True, separators=(",", ":"))
    return "plan-" + hashlib.sha256(canonical.encode()).hexdigest()[:40]


def item_cost(item):
    return CREDITS[item["endpoint"]] * item.get("count", 1)


# ------------------------------------------------------------------- ledger

class Ledger:
    """Append-only JSONL record of every credit-relevant event."""

    def __init__(self, path):
        self.path = Path(path)

    def entries(self):
        if not self.path.is_file():
            return []
        return [json.loads(line) for line in self.path.read_text().splitlines() if line.strip()]

    def record(self, event, **fields):
        entry = {"utc": dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds"),
                 "event": event, **fields}
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(entry, sort_keys=True) + "\n")
        return entry

    def submission(self, key):
        """The 'submitted' entry for an idempotency key, if any."""
        return next((e for e in self.entries() if e["event"] == "submitted" and e["key"] == key), None)

    def spent(self):
        """Credits committed: every intent counts until refunded (conservative)."""
        entries = self.entries()
        committed = {e["key"]: e["credits"] for e in entries if e["event"] == "intent"}
        refunded = {}
        for e in entries:  # a refund without credits is a full refund
            if e["event"] == "refunded" and e["key"] in committed:
                refunded[e["key"]] = refunded.get(e["key"], 0) + e.get("credits", committed[e["key"]])
        return sum(max(0, c - refunded.get(k, 0)) for k, c in committed.items())


def budget_status(plan, ledger):
    budget = plan["budget"]
    spent = ledger.spent()
    usable = budget["credits"] - budget["reserve_credits"]
    return {"credits": budget["credits"], "reserve_credits": budget["reserve_credits"],
            "spent_or_pending": spent, "usable_remaining": usable - spent,
            "expires": str(budget.get("expires")), "costs_source": COSTS_SOURCE}


# ------------------------------------------------------------------- client

TOKEN_FILE = Path.home() / ".config" / "projectsim" / "token"


def read_token_file(path=None):
    """Token from a private file (never the repo); refuses group/world-readable."""
    path = Path(path or os.environ.get("PROJECTSIM_TOKEN_FILE") or TOKEN_FILE)
    if not path.is_file():
        return None
    if path.stat().st_mode & 0o077:
        raise RuntimeError(f"{path} is readable by other users; run: chmod 600 {path}")
    return path.read_text(encoding="utf-8").strip() or None


class Client:
    def __init__(self, token=None, transport=None, base_url=BASE_URL):
        token = token or os.environ.get("PROJECTSIM_TOKEN") or read_token_file()
        if not token:
            raise RuntimeError(f"PROJECTSIM_TOKEN is not set and {TOKEN_FILE} does not exist")
        if not token.startswith("kps_"):
            raise RuntimeError("PROJECTSIM_TOKEN does not look like a ProjectSim token (kps_...)")
        self._http = httpx.Client(base_url=base_url, transport=transport, timeout=30.0,
                                  headers={"Authorization": f"Bearer {token}"})
        # Presigned storage URLs must be fetched WITHOUT the API token.
        self._download = httpx.Client(transport=transport, timeout=120.0, follow_redirects=True)

    def __repr__(self):
        return "<projectsim.Client token=***>"

    def _json(self, response, expected):
        if response.status_code != expected:
            try:
                problem = response.json()
            except ValueError:
                problem = {"title": response.text[:200]}
            raise ProjectSimError(response.status_code, problem)
        return response.json()

    def auth_check(self):
        """Free call: lists one asset to prove the token works."""
        return self._json(self._http.get("/v1/assets", params={"limit": 1}), 200)

    def create(self, endpoint, body, key):
        return self._json(self._http.post(ENDPOINTS[endpoint], json=body,
                                          headers={"Idempotency-Key": key}), 202)

    def job(self, job_id):
        return self._json(self._http.get(f"/v1/jobs/{job_id}"), 200)

    def asset(self, asset_id):
        return self._json(self._http.get(f"/v1/assets/{asset_id}"), 200)

    def variation_set(self, set_id):
        return self._json(self._http.get(f"/v1/variation-sets/{set_id}"), 200)

    def set_members(self, set_id):
        """All members (paginated by variant_index cursor), files included."""
        members, cursor = [], None
        while True:
            params = {"limit": 100, **({"cursor": cursor} if cursor is not None else {})}
            page = self._json(self._http.get(f"/v1/variation-sets/{set_id}/assets", params=params), 200)
            members += page["members"]
            cursor = page.get("next_cursor")
            if cursor is None:
                return members

    def download(self, asset, dest):
        """Download every file, preserving relative paths; return sha256 map."""
        dest = Path(dest).resolve()
        hashes = {}
        for f in asset.get("files", []):
            target = (dest / f["path"]).resolve()
            if not target.is_relative_to(dest):
                raise ValueError(f"refusing path outside download dir: {f['path']}")
            target.parent.mkdir(parents=True, exist_ok=True)
            h = hashlib.sha256()
            with self._download.stream("GET", f["url"]) as r:
                r.raise_for_status()
                with target.open("wb") as out:
                    for chunk in r.iter_bytes(1 << 20):
                        out.write(chunk)
                        h.update(chunk)
            hashes[f["path"]] = h.hexdigest()
        return hashes


# Preferred mesh / sidecar file kinds (AssetFileEntry.kind), best first.
GLB_KINDS = ("simready_glb", "glb")
SIDECAR_KINDS = ("simready_sidecar", "physics_sidecar")


def pick_files(metadata):
    """(glb path, sidecar path or None) from stored asset metadata."""
    files = metadata.get("files", [])
    by_kind = {}
    for f in files:
        by_kind.setdefault(f.get("kind"), f["path"])
    glb = next((by_kind[k] for k in GLB_KINDS if k in by_kind), None)
    if glb is None:
        raise ValueError(f"no GLB in asset files (kinds: {sorted(by_kind)}); "
                         "was it generated with physics.simulator=agnostic?")
    return glb, next((by_kind[k] for k in SIDECAR_KINDS if k in by_kind), None)


def strip_urls(obj):
    """Deep copy without presigned URLs (they are bearer credentials)."""
    if isinstance(obj, dict):
        return {k: strip_urls(v) for k, v in obj.items() if k != "url"}
    if isinstance(obj, list):
        return [strip_urls(v) for v in obj]
    return obj


# --------------------------------------------------------------- workflows

def submit(client, plan, item_id, ledger, confirm_spend=False):
    """Create one plan item. Refuses without confirm_spend or past budget."""
    item = next((i for i in plan["items"] if i["id"] == item_id), None)
    if item is None:
        raise KeyError(item_id)
    body = request_body(plan, item)
    if "TODO" in json.dumps(body):
        raise ValueError(f"{item_id} still has TODO placeholders in its request")
    key = idempotency_key(item["endpoint"], body)
    existing = ledger.submission(key)
    if existing:
        return existing  # already bought; never re-send
    cost = item_cost(item)
    if not confirm_spend:
        raise SpendNotConfirmed(f"{item_id} costs {cost} credits; pass confirm_spend after approval")
    status = budget_status(plan, ledger)
    if cost > status["usable_remaining"]:
        raise BudgetExceeded(f"{item_id} needs {cost}, only {status['usable_remaining']} "
                             f"usable (reserve {status['reserve_credits']} held back)")
    ledger.record("intent", key=key, item=item_id, credits=cost, endpoint=item["endpoint"])
    try:
        created = client.create(item["endpoint"], body, key)
    except ProjectSimError as err:
        # 4xx before creation (402/409/422/429) creates nothing -> nothing spent.
        if 400 <= err.status < 500:
            ledger.record("refunded", key=key, item=item_id, reason=f"rejected HTTP {err.status}")
        raise
    id_field = "set_id" if item["endpoint"] in SET_COUNT else "asset_id"
    return ledger.record("submitted", key=key, item=item_id, credits=cost,
                         job_id=created["job_id"], effective_params=created.get("effective_params"),
                         **{id_field: created["id"]})


def fetch(client, asset_id, raw_root, ledger):
    """Download a finished asset into raw_root/<asset_id>/ with metadata."""
    asset = client.asset(asset_id)
    status = (asset.get("job") or {}).get("status")  # assets have no status of their own
    sub = next((e for e in ledger.entries() if e.get("asset_id") == asset_id and e["event"] == "submitted"), None)
    if status in {"failed", "canceled"}:
        if sub and not any(e["event"] == "refunded" and e["key"] == sub["key"] for e in ledger.entries()):
            ledger.record("refunded", key=sub["key"], item=sub["item"], reason=f"job {status}")
        return {"status": status, "files": {}}
    if not asset.get("files"):
        return {"status": status or "pending", "files": {}}
    dest = Path(raw_root) / asset_id
    hashes = client.download(asset, dest)
    (dest / "projectsim_asset.json").write_text(json.dumps(strip_urls(asset), indent=2) + "\n")
    ledger.record("downloaded", asset_id=asset_id, item=sub["item"] if sub else None, files=hashes)
    return {"status": "complete", "files": hashes, "dir": str(dest)}


def fetch_set(client, set_id, raw_root, ledger):
    """Download a finished variation set: raw_root/<set_id>/<variant_index>/.

    Failed or canceled members are logged as per-member refunds, so the
    ledger's spent figure tracks what the docs say is actually charged."""
    detail = client.variation_set(set_id)
    status = detail.get("status")
    if status not in TERMINAL:
        return {"status": status or "pending", "counts": detail.get("counts")}
    sub = next((e for e in ledger.entries() if e.get("set_id") == set_id and e["event"] == "submitted"), None)
    members = client.set_members(set_id)
    dest = Path(raw_root) / set_id
    downloaded, not_built = {}, 0
    for m in members:
        if m.get("status") in {"failed", "canceled"} or not m.get("files"):
            not_built += 1
            continue
        folder = dest / f"{m['variant_index']:03d}"
        downloaded[m["variant_index"]] = client.download(m, folder)
        (folder / "projectsim_asset.json").write_text(json.dumps(strip_urls(m), indent=2) + "\n")
    (dest / "projectsim_set.json").write_text(json.dumps(strip_urls(detail), indent=2) + "\n")
    if sub and not_built and not any(e["event"] == "refunded" and e["key"] == sub["key"] for e in ledger.entries()):
        per = sub["credits"] // max(1, detail.get("count", len(members)))
        ledger.record("refunded", key=sub["key"], item=sub["item"], credits=per * not_built,
                      reason=f"{not_built} member(s) failed/canceled")
    ledger.record("downloaded", set_id=set_id, item=sub["item"] if sub else None,
                  members=len(downloaded), failed=not_built)
    return {"status": status, "members": len(downloaded), "failed": not_built, "dir": str(dest)}
