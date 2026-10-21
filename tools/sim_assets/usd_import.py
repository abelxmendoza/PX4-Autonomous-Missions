"""UsdPhysics asset (ProjectSim procedural / Isaac-style USD) -> Gazebo SDF.

Gazebo Harmonic has no USD loader on this machine (no usd2sdf, no Blender),
so this reads the stage with Pixar's usd-core (`pip install usd-core`) and
writes a multi-link SDF model directly:

  USD                                   SDF
  prim with PhysicsRigidBodyAPI     ->  <link> at the prim's stage pose
  Mesh, purpose default/render      ->  link visual (.gltf per link, textures shared)
  Mesh with PhysicsCollisionAPI     ->  link collision (one STL per link)
  PhysicsMassAPI mass/CoM/principal ->  <inertial> (pose rotation = principal axes)
  Revolute/Prismatic/FixedJoint     ->  <joint>, pose and axis expressed in __model__
  joint limits (deg for revolute)   ->  radians; unlimited -> revolute without <limit>
  drive damping, PhysX joint friction -> <dynamics damping/friction>

Two modes:
  static       every link frozen (joints written but immobile) and the USD
               collision mesh used as-is: a navigation obstacle
  articulated  dynamic links; collision per link is the convex hull of the
               USD collision mesh by default, because large non-convex
               triangle meshes on dynamic bodies are slow and unreliable in
               Gazebo's physics. The hull fills openings; see manifest warnings.

Joint conventions are cross-checked in tests against the MJCF ProjectSim
ships for the same asset (the hinge axis must match).
"""
from __future__ import annotations

import datetime as dt
import hashlib
import json
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh

from .gz_model import MANIFEST, _fmt, model_config, sha256
from .mesh_import import UP_ROTATIONS, rotation_to_rpy

try:
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade
except ImportError:  # optional dependency
    Usd = None

AXIS = {"X": np.array([1.0, 0, 0]), "Y": np.array([0, 1.0, 0]), "Z": np.array([0, 0, 1.0])}
JOINT_TYPES = {"PhysicsRevoluteJoint": "revolute", "PhysicsPrismaticJoint": "prismatic",
               "PhysicsFixedJoint": "fixed"}
TEXTURE_INPUTS = ("diffuseColor", "normal", "roughness", "metallic")


def _require_usd():
    if Usd is None:
        raise RuntimeError("USD import needs usd-core: pip install usd-core")


def _np(m):
    """Gf.Matrix4d (row-vector convention) -> 4x4 column-vector numpy."""
    return np.array(m, dtype=float).T


def _quat(q):
    """Gf.Quat (real, imaginary) -> 3x3 rotation."""
    w, (x, y, z) = q.GetReal(), q.GetImaginary()
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _pose(t):
    return list(t[:3, 3]) + list(rotation_to_rpy(t[:3, :3]))


def _triangles(mesh):
    """Fan-triangulate; returns (points, tri corner indices into faceVertexIndices)."""
    counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
    if (counts < 3).any():
        raise ValueError(f"{mesh.GetPath()}: face with < 3 vertices")
    starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
    corners = [np.stack([np.full(c - 2, s), s + np.arange(1, c - 1), s + np.arange(2, c)], axis=1)
               for s, c in zip(starts, counts)]
    return np.asarray(mesh.GetPointsAttr().Get(), dtype=float), np.concatenate(corners)


def _textures(prim, cache):
    """{input name: absolute texture path} from the bound UsdPreviewSurface."""
    material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
    if not material:
        return {}
    key = str(material.GetPath())
    if key not in cache:
        found = {}
        surface = material.ComputeSurfaceSource()[0]
        for name in TEXTURE_INPUTS:
            inp = surface.GetInput(name) if surface else None
            if inp and inp.HasConnectedSource():
                src = UsdShade.Shader(inp.GetConnectedSource()[0].GetPrim())
                f = src.GetInput("file")
                if f and f.Get():
                    found[name] = f.Get().resolvedPath
        cache[key] = found
    return cache[key]


def _pbr(textures, max_px, image_cache):
    from PIL import Image

    def load(name, mode):
        path = textures.get(name)
        if not path:
            return None
        if (path, mode) not in image_cache:
            img = Image.open(path).convert(mode)
            if max_px and max(img.size) > max_px:
                img = img.resize((max_px, max_px * img.size[1] // img.size[0]), Image.LANCZOS)
            image_cache[(path, mode)] = img
        return image_cache[(path, mode)]

    base, normal = load("diffuseColor", "RGBA"), load("normal", "RGB")
    rough, metal = load("roughness", "L"), load("metallic", "L")
    mr = None
    if rough is not None or metal is not None:
        size = (rough or metal).size
        zero = Image.new("L", size, 0)
        mr = Image.merge("RGB", (zero, rough or Image.new("L", size, 255), metal or zero))
    return trimesh.visual.material.PBRMaterial(
        baseColorTexture=base, normalTexture=normal, metallicRoughnessTexture=mr,
        metallicFactor=1.0 if metal is not None else 0.0, roughnessFactor=1.0)


def _visual_mesh(mesh, to_link, tex_cache, img_cache, max_px):
    points, corners = _triangles(mesh)
    fvi = np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
    tri_vi = fvi[corners]
    st = UsdGeom.PrimvarsAPI(mesh.GetPrim()).GetPrimvar("st")
    textures = _textures(mesh.GetPrim(), tex_cache)
    pts = trimesh.transform_points(points, to_link)
    if not (st and st.Get() is not None and textures):
        return trimesh.Trimesh(pts, tri_vi, process=False)
    uv = np.asarray(st.ComputeFlattened(), dtype=float)
    interp = st.GetInterpolation()
    if interp == "faceVarying":
        uv_c = uv[corners]
    elif interp in ("vertex", "varying"):
        uv_c = uv[tri_vi]
    else:
        return trimesh.Trimesh(pts, tri_vi, process=False)
    # unweld so each triangle corner carries its own UV
    verts = pts[tri_vi].reshape(-1, 3)
    faces = np.arange(len(verts)).reshape(-1, 3)
    visual = trimesh.visual.TextureVisuals(uv=uv_c.reshape(-1, 2),
                                           material=_pbr(textures, max_px, img_cache))
    return trimesh.Trimesh(verts, faces, visual=visual, process=False)


def _collision_mesh(mesh, to_link):
    points, corners = _triangles(mesh)
    fvi = np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
    return trimesh.Trimesh(trimesh.transform_points(points, to_link), fvi[corners], process=True)


def _owning_body(prim, bodies):
    p = prim
    while p and p.GetPath() != p.GetPath().absoluteRootPath:
        if str(p.GetPath()) in bodies:
            return str(p.GetPath())
        p = p.GetParent()
    return None


def read_stage(usd_path, max_texture_px=2048):
    """Parse bodies, meshes, inertia and joints into plain data (Gazebo frame)."""
    _require_usd()
    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise ValueError(f"cannot open {usd_path}")
    up = UsdGeom.GetStageUpAxis(stage).lower()
    if up not in UP_ROTATIONS:
        raise ValueError(f"unsupported upAxis {up}")
    fix = np.eye(4)
    fix[:3, :3] = UP_ROTATIONS[up] * UsdGeom.GetStageMetersPerUnit(stage)
    cache = UsdGeom.XformCache()
    warnings = []

    def world(prim):
        return fix @ _np(cache.GetLocalToWorldTransform(prim))

    def rigid(t):
        """Split a world transform into (rigid 4x4, uniform scale)."""
        r = t[:3, :3]
        s = np.cbrt(abs(np.linalg.det(r)))
        out = np.eye(4)
        out[:3, :3], out[:3, 3] = r / s, t[:3, 3]
        return out, s

    body_prims = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if not body_prims:
        default = stage.GetDefaultPrim() or stage.GetPseudoRoot()
        body_prims = [default]
        warnings.append("no PhysicsRigidBodyAPI: whole stage treated as one rigid link")
    bodies = {}
    for p in body_prims:
        t, s = rigid(world(p))
        bodies[str(p.GetPath())] = {"prim": p, "name": p.GetName(), "world": t,
                                    "visual": [], "collision": [], "mass": None}
    if len({b["name"] for b in bodies.values()}) != len(bodies):
        raise ValueError("duplicate rigid body names; cannot map to unique SDF links")

    tex_cache, img_cache = {}, {}
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        key = _owning_body(prim, bodies) or (next(iter(bodies)) if len(bodies) == 1 else None)
        if key is None:
            warnings.append(f"{prim.GetPath()}: mesh outside any rigid body, skipped")
            continue
        body = bodies[key]
        to_link = np.linalg.inv(body["world"]) @ world(prim)
        mesh = UsdGeom.Mesh(prim)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            body["collision"].append(_collision_mesh(mesh, to_link))
        elif UsdGeom.Imageable(prim).ComputePurpose() in ("default", "render"):
            body["visual"].append(_visual_mesh(mesh, to_link, tex_cache, img_cache, max_texture_px))

    for body in bodies.values():
        api = UsdPhysics.MassAPI(body["prim"])
        mass = api.GetMassAttr().Get() if api else None
        if mass:
            q = api.GetPrincipalAxesAttr().Get() or Gf.Quatf(1, 0, 0, 0)
            body["mass"] = {"mass_kg": float(mass),
                            "com": list(api.GetCenterOfMassAttr().Get() or (0, 0, 0)),
                            "principal_rpy": list(rotation_to_rpy(_quat(q))),
                            "diagonal": list(api.GetDiagonalInertiaAttr().Get() or (0, 0, 0))}

    joints = []
    for prim in stage.Traverse():
        kind = JOINT_TYPES.get(prim.GetTypeName())
        if kind is None:
            if prim.IsA(UsdPhysics.Joint):
                raise ValueError(f"{prim.GetPath()}: unsupported joint type {prim.GetTypeName()}")
            continue
        j = UsdPhysics.Joint(prim)
        b0 = [str(t) for t in j.GetBody0Rel().GetTargets()]
        b1 = [str(t) for t in j.GetBody1Rel().GetTargets()]
        if len(b1) != 1 or b1[0] not in bodies or (b0 and b0[0] not in bodies):
            raise ValueError(f"{prim.GetPath()}: joint bodies not resolvable to rigid bodies")
        frame0 = np.eye(4)
        frame0[:3, :3] = _quat(j.GetLocalRot0Attr().Get() or Gf.Quatf(1, 0, 0, 0))
        frame0[:3, 3] = np.asarray(j.GetLocalPos0Attr().Get() or (0, 0, 0)) * UsdGeom.GetStageMetersPerUnit(stage)
        parent_world = bodies[b0[0]]["world"] if b0 else np.eye(4)
        jworld = parent_world @ frame0
        entry = {"name": prim.GetName(), "type": kind,
                 "parent": bodies[b0[0]]["name"] if b0 else "world",
                 "child": bodies[b1[0]]["name"], "world": jworld}
        if kind != "fixed":
            axis_attr = prim.GetAttribute("physics:axis").Get() or "X"
            entry["axis"] = (jworld[:3, :3] @ AXIS[axis_attr]).tolist()
            lo = prim.GetAttribute("physics:lowerLimit").Get()
            hi = prim.GetAttribute("physics:upperLimit").Get()
            conv = math.radians if kind == "revolute" else (lambda v: v * UsdGeom.GetStageMetersPerUnit(stage))
            entry["limits"] = None if lo is None or hi is None or not math.isfinite(lo) else [conv(lo), conv(hi)]
            # Unlimited hinge (wheel, caster swivel): stay "revolute" with no
            # <limit> (SDF default is unbounded). gz-sim 8's dartsim plugin
            # turns type="continuous" into a FIXED joint (found by acceptance).
            entry["unlimited"] = entry["limits"] is None
            drive = "angular" if kind == "revolute" else "linear"
            damping = prim.GetAttribute(f"drive:{drive}:physics:damping").Get()
            friction = prim.GetAttribute(f"physxJointAxis:{drive}:staticFrictionEffort").Get()
            entry["damping"] = float(damping or 0.0)
            entry["friction"] = float(friction or 0.0)
        joints.append(entry)
    return {"bodies": list(bodies.values()), "joints": joints, "warnings": warnings,
            "up_axis": up, "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage)}


def export_visual(meshes, out_dir, link):
    """Write one link's visuals as .gltf with buffers in separate files.

    Texture buffers are named by content hash, so links sharing a material
    atlas (every part of a ProjectSim asset) share one copy on disk instead
    of each GLB embedding its own (a 3 MB caster fork, before this)."""
    from trimesh.exchange import gltf
    files = gltf.export_gltf(trimesh.Scene(meshes))
    doc = json.loads(files.pop("model.gltf"))
    image_buffers = {doc["bufferViews"][img["bufferView"]]["buffer"]
                     for img in doc.get("images", []) if "bufferView" in img}
    for i, buf in enumerate(doc.get("buffers", [])):
        data = files[buf["uri"]]
        if i in image_buffers:
            buf["uri"] = f"tex_{hashlib.sha256(data).hexdigest()[:16]}.bin"
        else:
            buf["uri"] = f"{link}_{i}.bin"
        target = Path(out_dir) / buf["uri"]
        if not target.exists():
            target.write_bytes(data)
    (Path(out_dir) / f"{link}.gltf").write_text(json.dumps(doc), encoding="utf-8")
    return f"{link}.gltf"


def model_sdf(name, data, static):
    sdf = ET.Element("sdf", version="1.9")
    model = ET.SubElement(sdf, "model", name=name)
    ET.SubElement(model, "static").text = "true" if static else "false"
    for b in data["bodies"]:
        link = ET.SubElement(model, "link", name=b["name"])
        ET.SubElement(link, "pose").text = _fmt(_pose(b["world"]))
        if b["mass"] and not static:
            inertial = ET.SubElement(link, "inertial")
            ET.SubElement(inertial, "pose").text = _fmt(b["mass"]["com"] + b["mass"]["principal_rpy"])
            ET.SubElement(inertial, "mass").text = f"{b['mass']['mass_kg']:.6g}"
            tensor = ET.SubElement(inertial, "inertia")
            for tag, v in zip(("ixx", "iyy", "izz"), b["mass"]["diagonal"]):
                ET.SubElement(tensor, tag).text = f"{v:.6g}"
            for tag in ("ixy", "ixz", "iyz"):
                ET.SubElement(tensor, tag).text = "0"
        if b.get("collision_file"):
            col = ET.SubElement(link, "collision", name="collision")
            mesh = ET.SubElement(ET.SubElement(col, "geometry"), "mesh")
            ET.SubElement(mesh, "uri").text = f"model://{name}/meshes/{b['collision_file']}"
        if b.get("visual_file"):
            vis = ET.SubElement(link, "visual", name="visual")
            mesh = ET.SubElement(ET.SubElement(vis, "geometry"), "mesh")
            ET.SubElement(mesh, "uri").text = f"model://{name}/meshes/{b['visual_file']}"
    for j in data["joints"]:
        joint = ET.SubElement(model, "joint", name=j["name"], type=j["type"])
        ET.SubElement(joint, "parent").text = j["parent"]
        ET.SubElement(joint, "child").text = j["child"]
        ET.SubElement(joint, "pose", relative_to="__model__").text = _fmt(_pose(j["world"]))
        if j["type"] == "fixed":
            continue
        axis = ET.SubElement(joint, "axis")
        ET.SubElement(axis, "xyz", expressed_in="__model__").text = _fmt(j["axis"])
        if j.get("limits"):
            limit = ET.SubElement(axis, "limit")
            ET.SubElement(limit, "lower").text = f"{j['limits'][0]:.6g}"
            ET.SubElement(limit, "upper").text = f"{j['limits'][1]:.6g}"
        dyn = ET.SubElement(axis, "dynamics")
        ET.SubElement(dyn, "damping").text = f"{j['damping']:.6g}"
        ET.SubElement(dyn, "friction").text = f"{j['friction']:.6g}"
    ET.indent(sdf, space="  ")
    return ET.tostring(sdf, encoding="unicode", xml_declaration=True) + "\n"


def convert(usd_path, out_root, name, mode="static", collision=None, provenance=None,
            max_texture_px=2048, overwrite=False):
    """Write models/NAME from a UsdPhysics asset; return the manifest."""
    if mode not in ("static", "articulated"):
        raise ValueError("mode must be static or articulated")
    collision = collision or ("usd" if mode == "static" else "convex_hull")
    if collision not in ("usd", "convex_hull", "box"):
        raise ValueError("collision must be usd, convex_hull or box")
    if not name or not all(c.isalnum() or c == "_" for c in name):
        raise ValueError("name must be non-empty [A-Za-z0-9_]")
    usd_path = Path(usd_path)
    data = read_stage(usd_path, max_texture_px)
    warnings = list(data["warnings"])

    model_dir = Path(out_root) / name
    if model_dir.exists():
        if not overwrite or not (model_dir / MANIFEST).is_file():
            raise FileExistsError(f"{model_dir} exists (generated models need overwrite=True; "
                                  "hand-made ones are never replaced)")
        shutil.rmtree(model_dir)
    meshes = model_dir / "meshes"
    meshes.mkdir(parents=True)

    all_visual = []
    links = []
    for b in data["bodies"]:
        if b["visual"]:
            b["visual_file"] = export_visual(b["visual"], meshes, b["name"])
            all_visual += [trimesh.Trimesh(trimesh.transform_points(v.vertices, b["world"]), v.faces,
                                           process=False) for v in b["visual"]]
        source = b["collision"] or b["visual"]
        if not b["collision"]:
            warnings.append(f"{b['name']}: no collision mesh in USD, derived from visual")
        col = None
        if source:
            merged = trimesh.util.concatenate([trimesh.Trimesh(m.vertices, m.faces) for m in source])
            col = {"usd": merged, "convex_hull": merged.convex_hull,
                   "box": merged.bounding_box_oriented.to_mesh()}[collision]
            b["collision_file"] = f"{b['name']}_collision.stl"
            col.export(meshes / b["collision_file"])
        if mode == "articulated" and not b["mass"]:
            raise ValueError(f"{b['name']}: no PhysicsMassAPI; articulated mode needs mass")
        links.append({"name": b["name"], "pose": _pose(b["world"]), "mass": b["mass"],
                      "visual_meshes": len(b["visual"]),
                      "visual_triangles": int(sum(len(v.faces) for v in b["visual"])),
                      "collision_triangles": int(len(col.faces)) if col is not None else 0})
    if collision == "convex_hull" and len(data["bodies"]) and mode == "articulated":
        warnings.append("convex_hull collision fills openings (wire mesh, gate gaps); "
                        "sensors see the visual mesh, contact does not")
    if mode == "static" and data["joints"]:
        warnings.append("static mode: joints are written but frozen at the authored pose")

    (model_dir / "model.config").write_text(
        model_config(name, (provenance or {}).get("description") or f"{name} (converted from USD)"),
        encoding="utf-8")
    (model_dir / "model.sdf").write_text(model_sdf(name, data, mode == "static"), encoding="utf-8")

    scene = trimesh.util.concatenate(all_visual) if all_visual else None
    lo, hi = (scene.bounds if scene is not None else (np.zeros(3), np.zeros(3)))
    if scene is not None and abs(lo[2]) > 0.05:
        warnings.append(f"lowest visual point at z={lo[2]:.3f} m, not on the ground plane")
    src_dir = usd_path.parent
    manifest = {
        "format": "sim-asset-manifest", "version": 1, "name": name,
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds"),
        "provenance": provenance or {},
        "source_file": usd_path.name,
        "source_sha256": {str(p.relative_to(src_dir)): sha256(p)
                          for p in sorted(src_dir.rglob("*")) if p.is_file()},
        "importer": "usd_import",
        "mode": mode, "collision": collision,
        "stage": {"up_axis": data["up_axis"], "meters_per_unit": data["meters_per_unit"]},
        "extents_m": (hi - lo).tolist(),
        "bounds_m": [lo.tolist(), hi.tolist()],
        "footprint_radius_m": float(np.hypot(*np.maximum(abs(lo[:2]), abs(hi[:2])))),
        "links": links,
        "joints": [{k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in j.items()
                    if k != "world"} | {"pose": _pose(j["world"])} for j in data["joints"]],
        "max_texture_px": max_texture_px,
        "inertial_status": "from USD PhysicsMassAPI (generator-computed, not measured)",
        "warnings": warnings,
    }
    (model_dir / MANIFEST).write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest
