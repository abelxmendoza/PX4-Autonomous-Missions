"""Contract tests for tools/sim_assets (mesh -> Gazebo model pipeline).

Fast tests need numpy + trimesh + httpx + PyYAML and no network: the
ProjectSim client runs against httpx.MockTransport, so no credits can be
spent. The drop test needs a real `gz` binary and is marked integration.
"""
import json
import math
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

trimesh = pytest.importorskip("trimesh")
httpx = pytest.importorskip("httpx")
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from sim_assets import gz_model, mesh_import, projectsim as ps, world_builder  # noqa: E402
from sim_assets.mesh_import import ImportSpec  # noqa: E402


def step_block_glb(path):
    """+Y-up two-level block in arbitrary units: x in [-1, 0] is 1.0 tall,
    x in [0, 1] is 0.4 tall, z (glTF depth) in [-0.25, 0.25]."""
    tall = trimesh.creation.box(extents=[1.0, 1.0, 0.5])
    tall.apply_translation([-0.5, 0.5, 0.0])
    low = trimesh.creation.box(extents=[1.0, 0.4, 0.5])
    low.apply_translation([0.5, 0.2, 0.0])
    trimesh.util.concatenate([tall, low]).export(path)
    return path


@pytest.fixture
def glb(tmp_path):
    return step_block_glb(tmp_path / "step_block.glb")


# ------------------------------------------------------------ mesh_import

def test_y_up_rotation_scale_and_origin(glb):
    spec = ImportSpec(name="step_block", up_axis="y", target_size_m=2.0)
    r = mesh_import.normalise(mesh_import.load_mesh(glb), spec)
    assert r.scale == pytest.approx(2.0)
    lo, hi = r.mesh.bounds
    np.testing.assert_allclose(lo, [-2.0, -0.5, 0.0], atol=1e-9)
    np.testing.assert_allclose(hi, [2.0, 0.5, 2.0], atol=1e-9)
    # glTF depth +z must become Gazebo -y (right-handed +90 deg about X)
    assert r.rotation @ np.array([0, 0, 1.0]) == pytest.approx([0, -1, 0])
    assert r.visual_rpy == pytest.approx((math.pi / 2, 0, 0))


def test_visual_pose_reproduces_normalised_mesh(glb):
    """The SDF visual pose+scale applied to the RAW file must give the same
    vertices as the analysed mesh, otherwise visual and collision diverge."""
    raw = mesh_import.load_mesh(glb)
    r = mesh_import.normalise(raw, ImportSpec(name="s", target_size_m=2.0))
    roll, pitch, yaw = r.visual_rpy
    rz = trimesh.transformations.euler_matrix(roll, pitch, yaw, "sxyz")[:3, :3]
    np.testing.assert_allclose(raw.vertices * r.scale @ rz.T + r.translation, r.mesh.vertices, atol=1e-9)


def test_z_up_and_known_units(glb):
    r = mesh_import.normalise(mesh_import.load_mesh(glb),
                              ImportSpec(name="s", up_axis="z", source_unit_m=1.0))
    assert r.scale == 1.0
    assert r.mesh.extents == pytest.approx([2.0, 1.0, 0.5])


@pytest.mark.parametrize("kwargs", [
    {"name": "bad name"},
    {"name": "x", "up_axis": "w"},
    {"name": "x", "collision": "sdf"},
    {"name": "x", "target_size_m": None},                     # no scale basis
    {"name": "x", "source_unit_m": 1.0},                      # both bases
    {"name": "x", "target_size_m": -1.0, "source_unit_m": None},
    {"name": "x", "target_size_m": float("nan")},
    {"name": "x", "static": False},                           # dynamic, no mass
])
def test_spec_rejects(kwargs):
    base = {"target_size_m": 1.0}
    base.update(kwargs)
    with pytest.raises(ValueError):
        ImportSpec(**base).validate()


def test_implausible_scale_rejected(glb):
    with pytest.raises(ValueError, match="implausible"):
        mesh_import.normalise(mesh_import.load_mesh(glb), ImportSpec(name="s", source_unit_m=1000.0))


def test_collision_budget_enforced(glb):
    spec = ImportSpec(name="s", target_size_m=1.0, max_collision_tris=4)
    with pytest.raises(ValueError, match="collision budget"):
        mesh_import.normalise(mesh_import.load_mesh(glb), spec)


def test_convex_hull_warns_about_filled_gaps(glb):
    r = mesh_import.normalise(mesh_import.load_mesh(glb),
                              ImportSpec(name="s", target_size_m=1.0, collision="convex_hull"))
    assert any("fills gaps" in w for w in r.warnings)


def test_dynamic_inertia_is_labelled_assumed():
    box = trimesh.creation.box(extents=[2.0, 1.0, 0.5])  # z-up, watertight
    spec = ImportSpec(name="crate", up_axis="z", source_unit_m=1.0, static=False, density_kg_m3=100.0)
    r = mesh_import.normalise(box, spec)
    i = r.inertial
    assert i["mass_kg"] == pytest.approx(100.0)  # 1 m^3 x 100
    assert "ASSUMED" in i["status"] and "watertight" in i["volume_source"]
    m, (a, b, c) = 100.0, (2.0, 1.0, 0.5)
    assert i["inertia"][0][0] == pytest.approx(m * (b * b + c * c) / 12)
    assert i["inertia"][2][2] == pytest.approx(m * (a * a + b * b) / 12)
    assert i["center_of_mass"] == pytest.approx([0, 0, 0.25])


# --------------------------------------------------------------- gz_model

def _write(tmp_path, glb, **kw):
    spec = ImportSpec(name="step_block", target_size_m=2.0, **kw)
    result = mesh_import.normalise(mesh_import.load_mesh(glb), spec)
    out = tmp_path / "models"
    manifest = gz_model.write_model(out, glb, spec, result, {"source": "local", "license": "test"})
    return out / "step_block", manifest


def test_model_directory_contract(tmp_path, glb):
    model_dir, manifest = _write(tmp_path, glb)
    root = ET.parse(model_dir / "model.sdf").getroot()
    assert root.get("version") == "1.9"
    assert root.find("./model/static").text == "true"
    assert root.find("./model/link/visual/geometry/mesh/uri").text == "model://step_block/meshes/step_block.glb"
    assert root.find("./model/link/collision/geometry/mesh/uri").text == "model://step_block/meshes/collision.stl"
    assert root.find("./model/link/inertial") is None  # static
    assert (model_dir / "meshes" / "step_block.glb").read_bytes() == glb.read_bytes()  # byte-identical
    assert manifest["source_sha256"]["step_block.glb"] == gz_model.sha256(glb)
    assert "unmeasured" in manifest["transform"]["scale_basis"]
    assert manifest["footprint_radius_m"] == pytest.approx(math.hypot(4.0, 1.0) / 2)
    # the stored collision mesh is already in the Gazebo frame
    col = trimesh.load(model_dir / "meshes" / "collision.stl")
    assert col.bounds[1] == pytest.approx([2.0, 0.5, 2.0])


def test_box_collision(tmp_path, glb):
    model_dir, _ = _write(tmp_path, glb, collision="box")
    col = ET.parse(model_dir / "model.sdf").getroot().find("./model/link/collision")
    assert col.find("./geometry/box/size").text == "4 1 2"
    assert col.find("./pose").text == "0 0 1 0 0 0"
    assert not (model_dir / "meshes" / "collision.stl").exists()


def test_refuses_to_clobber_hand_made_model(tmp_path, glb):
    (tmp_path / "models" / "step_block").mkdir(parents=True)
    spec = ImportSpec(name="step_block", target_size_m=2.0)
    result = mesh_import.normalise(mesh_import.load_mesh(glb), spec)
    with pytest.raises(FileExistsError):
        gz_model.write_model(tmp_path / "models", glb, spec, result, {}, overwrite=True)


# ----------------------------------------------------------- world_builder

def scenario(**kw):
    data = {"format": "sim-scenario", "version": 1, "name": "t", "seed": 7,
            "area": {"x": [-10, 10], "y": [0, 30]}, "min_clearance_m": 1.0,
            "keepout": [{"x": 0, "y": 0, "radius_m": 3.0}],
            "placements": [{"model": "pole", "count": 6, "footprint_radius_m": 0.5},
                           {"model": "rack", "x": 0, "y": 15, "yaw": 0, "footprint_radius_m": 2.0}]}
    data.update(kw)
    return data


def test_world_is_reproducible_and_respects_constraints(tmp_path):
    a, rec_a = world_builder.build(scenario(), tmp_path)
    b, _ = world_builder.build(scenario(), tmp_path)
    c, _ = world_builder.build(scenario(seed=8), tmp_path)
    assert a == b and a != c
    inst = rec_a["instances"]
    assert inst[0]["name"] == "rack_0" and (inst[0]["x"], inst[0]["y"]) == (0, 15)  # fixed first
    for i, p in enumerate(inst):
        assert -10 + p["radius_m"] <= p["x"] <= 10 - p["radius_m"]
        assert math.hypot(p["x"], p["y"]) >= 3.0 + p["radius_m"]
        for q in inst[i + 1:]:
            assert math.hypot(p["x"] - q["x"], p["y"] - q["y"]) >= p["radius_m"] + q["radius_m"] + 1.0 - 1e-3
    world = ET.fromstring(a).find("./world")
    assert [e.find("uri").text for e in world.findall("include")].count("model://pole") == 6
    assert world.find("./physics/max_step_size").text == "0.001"  # PX4 conventions reused


def test_world_reads_footprint_from_manifest(tmp_path, glb):
    _write(tmp_path, glb)
    data = scenario(placements=[{"model": "step_block", "count": 2}])
    _, rec = world_builder.build(data, tmp_path / "models")
    assert rec["instances"][0]["radius_m"] == pytest.approx(math.hypot(4.0, 1.0) / 2)
    assert "step_block" in rec["manifest_sha256"]


@pytest.mark.parametrize("bad", [
    {"seed": "7"}, {"name": "has space"}, {"area": {"x": [1, 0], "y": [0, 1]}},
    {"placements": []}, {"placements": [{"model": "p", "x": 1}]},
    {"placements": [{"model": "p", "count": 0, "footprint_radius_m": 1}]},
])
def test_world_rejects_bad_scenarios(tmp_path, bad):
    with pytest.raises(ValueError):
        world_builder.build(scenario(**bad), tmp_path)


def test_world_reports_overcrowding(tmp_path):
    data = scenario(placements=[{"model": "p", "count": 500, "footprint_radius_m": 2.0}])
    with pytest.raises(ValueError, match="too crowded"):
        world_builder.build(data, tmp_path)


def test_fixed_placement_in_keepout_rejected(tmp_path):
    data = scenario(placements=[{"model": "p", "x": 0, "y": 1, "footprint_radius_m": 1}])
    with pytest.raises(ValueError, match="collides"):
        world_builder.build(data, tmp_path)


# -------------------------------------------------------------- projectsim

TOKEN = "kps_test_" + "x" * 24


def plan(credits=12, reserve=4):
    return {"format": "projectsim-plan", "version": 1,
            "budget": {"credits": credits, "reserve_credits": reserve},
            "defaults": {"standard": {"params": {"physics": {
                "simulator": "agnostic", "articulated": False, "collision_granularity": None}}},
                "procedural": {"params": {"physics": {"body": "rigid"}}}},
            "items": [{"id": "pole", "endpoint": "standard", "inputs": {"text": "a pole"}},
                      {"id": "rack", "endpoint": "standard", "inputs": {"text": "a rack"},
                       "params": {"mesh": {"quality": "low"}}},
                      {"id": "bays", "endpoint": "procedural", "count": 10,
                       "seed": {"text": "a shelving bay"}}]}


class FakeAPI:
    def __init__(self, status=202):
        self.calls = []
        self.status = status

    def __call__(self, request):
        self.calls.append(request)
        if request.method == "POST":
            if self.status != 202:
                return httpx.Response(self.status, json={"title": "nope", "status": self.status})
            return httpx.Response(202, json={"id": "ast_1", "job_id": "job_1", "status": "queued",
                                             "effective_params": {}})
        if request.url.path == "/v1/assets/ast_1":
            return httpx.Response(200, json={
                "id": "ast_1", "kind": "object_reconstruction", "created_at": "x",
                "effective_params": {}, "job": {"job_id": "job_1", "status": "complete"},
                "files": [{"kind": "glb", "path": "assets/pole/pole.glb",
                           "url": "https://storage.example/pole.glb?sig=SECRET", "expires_at": "x"}]})
        if request.url.host == "storage.example":
            assert "authorization" not in request.headers  # token never sent to storage
            return httpx.Response(200, content=b"glb-bytes")
        return httpx.Response(404, json={"title": "not found"})


def client(api):
    return ps.Client(token=TOKEN, transport=httpx.MockTransport(api))


def test_submit_without_confirmation_sends_nothing(tmp_path):
    api, ledger = FakeAPI(), ps.Ledger(tmp_path / "l.jsonl")
    with pytest.raises(ps.SpendNotConfirmed):
        ps.submit(client(api), plan(), "pole", ledger)
    assert api.calls == [] and ledger.entries() == []


def test_submit_records_intent_and_is_idempotent(tmp_path):
    api, ledger = FakeAPI(), ps.Ledger(tmp_path / "l.jsonl")
    entry = ps.submit(client(api), plan(), "pole", ledger, confirm_spend=True)
    assert entry["asset_id"] == "ast_1" and ledger.spent() == 4
    sent = json.loads(api.calls[0].content)
    assert sent["params"]["physics"] == {"simulator": "agnostic", "articulated": False,
                                         "collision_granularity": None}
    assert api.calls[0].headers["idempotency-key"].startswith("plan-")
    assert [e["event"] for e in ledger.entries()] == ["intent", "submitted"]
    ps.submit(client(api), plan(), "pole", ledger, confirm_spend=True)  # retry
    assert len(api.calls) == 1 and ledger.spent() == 4


def test_budget_reserve_is_enforced(tmp_path):
    api, ledger = FakeAPI(), ps.Ledger(tmp_path / "l.jsonl")
    p = plan(credits=8, reserve=4)
    ps.submit(client(api), p, "pole", ledger, confirm_spend=True)
    with pytest.raises(ps.BudgetExceeded):
        ps.submit(client(api), p, "rack", ledger, confirm_spend=True)
    assert len(api.calls) == 1


def test_rejected_request_is_refunded_in_ledger(tmp_path):
    api, ledger = FakeAPI(status=422), ps.Ledger(tmp_path / "l.jsonl")
    with pytest.raises(ps.ProjectSimError):
        ps.submit(client(api), plan(), "pole", ledger, confirm_spend=True)
    assert ledger.spent() == 0


def test_fetch_downloads_and_strips_presigned_urls(tmp_path):
    api, ledger = FakeAPI(), ps.Ledger(tmp_path / "l.jsonl")
    ps.submit(client(api), plan(), "pole", ledger, confirm_spend=True)
    out = ps.fetch(client(api), "ast_1", tmp_path / "raw", ledger)
    assert out["status"] == "complete"
    assert (tmp_path / "raw/ast_1/assets/pole/pole.glb").read_bytes() == b"glb-bytes"
    stored = (tmp_path / "raw/ast_1/projectsim_asset.json").read_text()
    assert "SECRET" not in stored and "storage.example" not in stored
    assert ps.pick_files(json.loads(stored)) == ("assets/pole/pole.glb", None)


def test_download_refuses_path_traversal(tmp_path):
    asset = {"files": [{"path": "../../evil", "url": "https://storage.example/x"}]}
    with pytest.raises(ValueError, match="outside"):
        client(FakeAPI()).download(asset, tmp_path / "raw")


def test_token_never_leaks(tmp_path, monkeypatch):
    monkeypatch.delenv("PROJECTSIM_TOKEN", raising=False)
    monkeypatch.setenv("PROJECTSIM_TOKEN_FILE", str(tmp_path / "missing"))
    with pytest.raises(RuntimeError, match="not set"):
        ps.Client()
    with pytest.raises(RuntimeError, match="kps_"):
        ps.Client(token="sk-wrong")
    api, ledger = FakeAPI(), ps.Ledger(tmp_path / "l.jsonl")
    c = client(api)
    ps.submit(c, plan(), "pole", ledger, confirm_spend=True)
    assert TOKEN not in repr(c) and TOKEN not in (tmp_path / "l.jsonl").read_text()


def test_repo_plan_is_valid_and_within_budget():
    root = Path(__file__).resolve().parents[1]
    p = ps.load_plan(root / "sim_assets/plans/initial_batch.yaml")
    total = sum(ps.item_cost(i) for i in p["items"])
    assert total <= p["budget"]["credits"] - p["budget"]["reserve_credits"]
    for item in p["items"]:
        settings = dict(item["import"])
        body = ps.request_body(p, item)
        if settings.pop("kind") == "usd":
            assert item["endpoint"] == "procedural" and "mesh" not in body.get("params", {})
        else:
            ImportSpec(name=item["id"], **settings).validate()
            assert body["params"]["physics"]["simulator"] == "agnostic"
    experiments = {i["experiment"]: ps.item_cost(i) for i in p["items"] if "experiment" in i}
    assert experiments == {"E1": 10, "E2": 4, "E3": 4}


def test_procedural_cost_body_and_limits():
    p = plan(credits=100, reserve=0)
    bays = p["items"][2]
    assert ps.item_cost(bays) == 10
    body = ps.request_body(p, bays)
    assert body == {"name": "bays", "count": 10, "seed": {"text": "a shelving bay"},
                    "params": {"physics": {"body": "rigid"}}}  # no standard-only defaults leak in
    for bad in ({**bays, "count": 9}, {**bays, "count": 501}, {k: v for k, v in bays.items() if k != "seed"}):
        with pytest.raises(ValueError):
            ps.load_plan_dict({**p, "items": [bad]})


def test_todo_placeholders_never_sent(tmp_path):
    api, ledger = FakeAPI(), ps.Ledger(tmp_path / "l.jsonl")
    p = plan()
    p["items"].append({"id": "m", "endpoint": "measured",
                       "inputs": {"images": [{"upload_id": "TODO"}], "capture": [{"upload_id": "TODO"}]}})
    with pytest.raises(ValueError, match="TODO"):
        ps.submit(client(api), p, "m", ledger, confirm_spend=True)
    assert api.calls == [] and ledger.entries() == []


def test_set_fetch_refunds_failed_members(tmp_path):
    def api(request):
        path = request.url.path
        if request.method == "POST":
            return httpx.Response(202, json={"id": "set_1", "job_id": "job_s", "status": "queued",
                                             "method": "procedural", "count": 10})
        if path == "/v1/variation-sets/set_1":
            return httpx.Response(200, json={"id": "set_1", "count": 10, "status": "partial",
                                             "counts": {}, "bundle": {}, "created_at": "x"})
        if path == "/v1/variation-sets/set_1/assets":
            members = [{"variant_index": i, "asset_id": f"a{i}", "status": "complete" if i <= 8 else "failed",
                        "files": [{"kind": "usd", "path": f"m{i}/m.usda",
                                   "url": f"https://storage.example/{i}?sig=S"}] if i <= 8 else []}
                       for i in range(1, 11)]
            return httpx.Response(200, json={"members": members, "next_cursor": None})
        return httpx.Response(200, content=b"usd")
    ledger = ps.Ledger(tmp_path / "l.jsonl")
    c = ps.Client(token=TOKEN, transport=httpx.MockTransport(api))
    p = plan(credits=100, reserve=0)
    entry = ps.submit(c, p, "bays", ledger, confirm_spend=True)
    assert entry["set_id"] == "set_1" and ledger.spent() == 10
    out = ps.fetch_set(c, "set_1", tmp_path / "raw", ledger)
    assert (out["members"], out["failed"]) == (8, 2)
    assert ledger.spent() == 8  # two failed members refunded at 1 credit each
    assert (tmp_path / "raw/set_1/003/m3/m.usda").read_bytes() == b"usd"
    assert "sig=S" not in (tmp_path / "raw/set_1/003/projectsim_asset.json").read_text()


# ------------------------------------------------------- Gazebo (integration)

def gate_glb(path):
    """+Y-up: two 1x1x0.5 towers at x = +/-1.5 with a 2-unit gap between."""
    towers = []
    for x in (-1.5, 1.5):
        t = trimesh.creation.box(extents=[1.0, 1.0, 0.5])
        t.apply_translation([x, 0.5, 0.0])
        towers.append(t)
    trimesh.util.concatenate(towers).export(path)
    return path


needs_gz = pytest.mark.skipif(shutil.which("gz") is None, reason="needs Gazebo (gz)")


@pytest.mark.integration
@needs_gz
def test_gazebo_orientation_and_scale(tmp_path, glb):
    """Real gz-sim: the step block stands upright at x2 scale (2.0 m / 0.8 m)."""
    from sim_assets import drop_test
    _write(tmp_path, glb)
    report = drop_test.run(tmp_path / "models", "step_block", [(-1.0, 0.0), (1.0, 0.0)])
    assert report["loaded"] and report["settled"], report
    tall, low = (p["surface_z"] for p in report["probes"])
    assert tall == pytest.approx(2.0, abs=0.03)  # lying on its side would read 0.5
    assert low == pytest.approx(0.8, abs=0.03)


@pytest.mark.integration
@needs_gz
@pytest.mark.parametrize("collision, gap_surface", [("mesh", 0.0), ("convex_hull", 2.0)])
def test_gazebo_gap_survives_mesh_collision_only(tmp_path, collision, gap_surface):
    """Real gz-sim: a probe between the towers reaches the ground with mesh
    collision; a convex hull bridges the gap, which is why racks, chairs
    and gates must not default to convex_hull."""
    from sim_assets import drop_test
    src = gate_glb(tmp_path / "gate.glb")
    spec = ImportSpec(name="gate", target_size_m=2.0, collision=collision)
    result = mesh_import.normalise(mesh_import.load_mesh(src), spec)
    gz_model.write_model(tmp_path / "models", src, spec, result, {"source": "test"})
    report = drop_test.run(tmp_path / "models", "gate", [(0.0, 0.0)])
    assert report["loaded"] and report["settled"], report
    assert report["probes"][0]["surface_z"] == pytest.approx(gap_surface, abs=0.03)


# ------------------------------------------------------------- usd_import

def usd_door_stage(path, up="Z", mpu=1.0, texture=None):
    """Two-link UsdPhysics asset shaped like ProjectSim procedural output:
    a 1x1x1 'frame' box at the origin and a 'door' link at (0.5, -0.5, 0)
    hinged by a revolute joint whose frame is +90 deg about Y with axis X
    (= world -Z, the convention seen in the projectsim cage sample)."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, up)
    UsdGeom.SetStageMetersPerUnit(stage, mpu)
    root = UsdGeom.Xform.Define(stage, "/asset")
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    box = trimesh.creation.box(extents=[1, 1, 1])
    box.apply_translation([0, 0, 0.5])

    def mesh(at, guide):
        m = UsdGeom.Mesh.Define(stage, at)
        m.CreatePointsAttr([Gf.Vec3f(*v) for v in box.vertices])
        m.CreateFaceVertexCountsAttr([3] * len(box.faces))
        m.CreateFaceVertexIndicesAttr(box.faces.flatten().tolist())
        if guide:
            UsdGeom.Imageable(m).CreatePurposeAttr("guide")
            UsdPhysics.CollisionAPI.Apply(m.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim())
        return m

    for body, offset, mass in (("frame", (0, 0, 0), 10.0), ("door", (0.5, -0.5, 0), 2.0)):
        x = UsdGeom.Xform.Define(stage, f"/asset/{body}")
        x.AddTranslateOp().Set(Gf.Vec3d(*offset))
        UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
        mapi = UsdPhysics.MassAPI.Apply(x.GetPrim())
        mapi.CreateMassAttr(mass)
        mapi.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0.5))
        mapi.CreateDiagonalInertiaAttr(Gf.Vec3f(1, 2, 3))
        mapi.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))
        vis = mesh(f"/asset/{body}/vis", guide=False)
        mesh(f"/asset/{body}/collision", guide=True)
        if texture is not None and body == "frame":
            st = UsdGeom.PrimvarsAPI(vis).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                                        UsdGeom.Tokens.faceVarying)
            st.Set([Gf.Vec2f(0.25, 0.75)] * (3 * len(box.faces)))
            mat = UsdShade.Material.Define(stage, "/asset/mat")
            surf = UsdShade.Shader.Define(stage, "/asset/mat/surface")
            surf.CreateIdAttr("UsdPreviewSurface")
            tex = UsdShade.Shader.Define(stage, "/asset/mat/tex")
            tex.CreateIdAttr("UsdUVTexture")
            tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(texture))
            surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
                tex.ConnectableAPI(), "rgb")
            mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI.Apply(vis.GetPrim()).Bind(mat)
    joint = UsdPhysics.RevoluteJoint.Define(stage, "/asset/hinge")
    joint.CreateBody0Rel().SetTargets(["/asset/frame"])
    joint.CreateBody1Rel().SetTargets(["/asset/door"])
    joint.CreateAxisAttr("X")
    joint.CreateLocalPos0Attr(Gf.Vec3f(0.5, -0.5, 0))
    joint.CreateLocalRot0Attr(Gf.Quatf(0.70710677, 0, 0.70710677, 0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot1Attr(Gf.Quatf(0.70710677, 0, 0.70710677, 0))
    joint.CreateLowerLimitAttr(0.0)
    joint.CreateUpperLimitAttr(90.0)
    stage.GetRootLayer().Save()
    return path


@pytest.fixture
def usd_import():
    pytest.importorskip("pxr")
    from sim_assets import usd_import as module
    return module


def test_usd_joint_axis_limits_and_links(tmp_path, usd_import):
    src = usd_door_stage(tmp_path / "door.usda")
    m = usd_import.convert(src, tmp_path / "models", "door_asset", mode="articulated")
    (joint,) = m["joints"]
    assert joint["axis"] == pytest.approx([0, 0, -1], abs=1e-6)
    assert joint["limits"] == pytest.approx([0, math.pi / 2])
    assert (joint["parent"], joint["child"]) == ("frame", "door")
    root = ET.parse(tmp_path / "models/door_asset/model.sdf").getroot()
    assert root.find("./model/static").text == "false"
    assert root.find("./model/link[@name='door']/pose").text.split()[:3] == ["0.5", "-0.5", "0"]
    assert root.find("./model/link[@name='door']/inertial/mass").text == "2"
    assert root.find("./model/joint/axis/xyz").get("expressed_in") == "__model__"
    assert m["extents_m"] == pytest.approx([1.5, 1.5, 1.0])
    assert any("fills openings" in w for w in m["warnings"])  # articulated default = hull


def test_usd_static_keeps_usd_collision_and_drops_inertia(tmp_path, usd_import):
    src = usd_door_stage(tmp_path / "door.usda")
    m = usd_import.convert(src, tmp_path / "models", "door_static")
    assert (m["mode"], m["collision"]) == ("static", "usd")
    root = ET.parse(tmp_path / "models/door_static/model.sdf").getroot()
    assert root.find("./model/static").text == "true"
    assert root.find(".//inertial") is None
    assert m["links"][0]["collision_triangles"] == 12


def test_usd_y_up_centimetres(tmp_path, usd_import):
    src = usd_door_stage(tmp_path / "door.usda", up="Y", mpu=0.01)
    m = usd_import.convert(src, tmp_path / "models", "door_cm")
    lo, hi = np.array(m["bounds_m"])
    # Y-up: the door's -0.5 offset is now vertical, so frame+door span
    # 1.5 units = 1.5 cm along +Z at 1 cm/unit
    assert (hi - lo)[2] == pytest.approx(0.015)
    assert m["joints"][0]["axis"] == pytest.approx([0, 1, 0], abs=1e-6)  # Rx(90) applied to -Z


def test_usd_textures_survive_into_glb(tmp_path, usd_import):
    from PIL import Image
    png = tmp_path / "red.png"
    Image.new("RGB", (8, 8), (255, 0, 0)).save(png)
    src = usd_door_stage(tmp_path / "door.usda", texture=png)
    usd_import.convert(src, tmp_path / "models", "door_tex")
    glb = trimesh.load(tmp_path / "models/door_tex/meshes/frame.gltf", force="scene")
    (geom,) = glb.geometry.values()
    material = geom.visual.material
    assert material.baseColorTexture is not None
    assert material.baseColorTexture.getpixel((0, 0))[:3] == (255, 0, 0)


@pytest.mark.integration
@needs_gz
def test_gazebo_loads_usd_conversion(tmp_path, usd_import):
    from sim_assets import drop_test
    usd_import.convert(usd_door_stage(tmp_path / "door.usda"), tmp_path / "models",
                       "door_static")
    report = drop_test.run(tmp_path / "models", "door_static", [(0.0, 0.0), (0.5, -0.5)])
    assert report["loaded"] and report["settled"], report
    assert [p["surface_z"] for p in report["probes"]] == pytest.approx([1.0, 1.0], abs=0.03)


def test_usd_unlimited_hinge_stays_revolute_without_limit(tmp_path, usd_import):
    """gz-sim 8 dartsim turns type="continuous" into FIXED, so an unlimited
    USD hinge (wheel, caster) must be a revolute joint with no <limit>."""
    from pxr import Usd, UsdPhysics
    src = usd_door_stage(tmp_path / "door.usda")
    stage = Usd.Stage.Open(str(src))
    joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath("/asset/hinge"))
    joint.GetLowerLimitAttr().Clear()
    joint.GetUpperLimitAttr().Clear()
    stage.GetRootLayer().Save()
    m = usd_import.convert(src, tmp_path / "models", "wheel", mode="articulated")
    assert m["joints"][0]["type"] == "revolute" and m["joints"][0]["unlimited"]
    root = ET.parse(tmp_path / "models/wheel/model.sdf").getroot()
    assert root.find("./model/joint").get("type") == "revolute"
    assert root.find("./model/joint/axis/limit") is None
    from sim_assets import acceptance
    joints = [c for c in acceptance.static_checks(m) if c["criterion"] == "joints_well_formed"]
    assert joints[0]["result"] == "PASS"


def test_token_file_must_be_private(tmp_path, monkeypatch):
    monkeypatch.delenv("PROJECTSIM_TOKEN", raising=False)
    f = tmp_path / "token"
    f.write_text(TOKEN + "\n")
    monkeypatch.setenv("PROJECTSIM_TOKEN_FILE", str(f))
    f.chmod(0o644)
    with pytest.raises(RuntimeError, match="chmod 600"):
        ps.Client()
    f.chmod(0o600)
    assert ps.read_token_file() == TOKEN
    ps.Client()  # constructs without error
