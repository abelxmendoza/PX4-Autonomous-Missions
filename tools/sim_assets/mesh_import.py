"""Normalise a source mesh into Gazebo's frame and derive physics data.

Frame contract (verified empirically on gz-sim 8.15, see
docs/sim_asset_pipeline.md): Gazebo loads GLB/STL vertex coordinates
verbatim and does NOT convert glTF's +Y-up to +Z-up. A +Y-up source is
therefore rotated +90 deg about X: (x, y, z) -> (x, -z, y), the same axis
map tools/realmforge_to_sdf.py uses.

After normalisation the model origin sits at the footprint centre on the
ground: x/y centred on the bounding box, base at z = 0.

Scale: generated (unmeasured) meshes have arbitrary units, so a real-world
target size along one axis is REQUIRED unless the caller states the source
unit explicitly. Mass and inertia are always assumptions (density x volume
or a stated mass) and are labelled as such in the manifest.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
import trimesh

UP_ROTATIONS = {
    "z": np.eye(3),
    # glTF / most DCC exports: +Y up -> rotate +90 deg about X.
    "y": np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]),
}
COLLISION_MODES = ("mesh", "convex_hull", "box")
AXES = {"x": 0, "y": 1, "z": 2}
# Sanity bounds for a navigation obstacle after scaling.
MIN_EXTENT_M = 0.01
MAX_EXTENT_M = 200.0
DEFAULT_MAX_COLLISION_TRIS = 20000


@dataclass
class ImportSpec:
    name: str
    up_axis: str = "y"
    target_axis: str | None = "z"
    target_size_m: float | None = None
    source_unit_m: float | None = None
    collision: str = "mesh"
    static: bool = True
    mass_kg: float | None = None
    density_kg_m3: float | None = None
    max_collision_tris: int = DEFAULT_MAX_COLLISION_TRIS

    def validate(self):
        if not self.name or not all(c.isalnum() or c == "_" for c in self.name):
            raise ValueError("name must be non-empty [A-Za-z0-9_]")
        if self.up_axis not in UP_ROTATIONS:
            raise ValueError(f"up_axis must be one of {sorted(UP_ROTATIONS)}")
        if self.collision not in COLLISION_MODES:
            raise ValueError(f"collision must be one of {COLLISION_MODES}")
        if (self.target_size_m is None) == (self.source_unit_m is None):
            raise ValueError("give exactly one of target_size_m (generated / "
                             "unmeasured mesh) or source_unit_m (known units)")
        for label, value in (("target_size_m", self.target_size_m),
                             ("source_unit_m", self.source_unit_m),
                             ("mass_kg", self.mass_kg),
                             ("density_kg_m3", self.density_kg_m3)):
            if value is not None and not (math.isfinite(value) and value > 0):
                raise ValueError(f"{label} must be a positive finite number")
        if self.target_size_m is not None and self.target_axis not in AXES:
            raise ValueError("target_axis must be x, y or z")
        if not self.static and self.mass_kg is None and self.density_kg_m3 is None:
            raise ValueError("a dynamic model needs mass_kg or density_kg_m3")


@dataclass
class ImportResult:
    rotation: np.ndarray            # 3x3, source -> Gazebo
    scale: float                    # uniform
    translation: np.ndarray         # applied after rotation and scale
    mesh: trimesh.Trimesh           # normalised visual geometry (analysis copy)
    collision_mesh: trimesh.Trimesh | None
    collision_box: tuple | None     # (size xyz, centre xyz)
    checks: dict
    inertial: dict | None
    warnings: list = field(default_factory=list)

    @property
    def visual_rpy(self):
        return rotation_to_rpy(self.rotation)


def load_mesh(path):
    """Load any trimesh-supported file, flattening the scene graph."""
    loaded = trimesh.load(str(path), force="scene")
    mesh = loaded.to_mesh() if isinstance(loaded, trimesh.Scene) else loaded
    if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
        raise ValueError(f"{path}: no triangle geometry found")
    return mesh


def rotation_to_rpy(r):
    """ZYX Euler angles (Gazebo/SDF pose convention) of a rotation matrix."""
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(r[2, 1], r[2, 2])
        yaw = math.atan2(r[1, 0], r[0, 0])
    else:  # gimbal lock: fold yaw into roll
        roll = math.atan2(-r[1, 2], r[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def mesh_checks(mesh):
    finite = bool(np.isfinite(mesh.vertices).all())
    extents = mesh.extents.tolist() if finite else None
    areas = mesh.area_faces
    return {
        "triangles": int(len(mesh.faces)),
        "vertices": int(len(mesh.vertices)),
        "finite_vertices": finite,
        "watertight": bool(mesh.is_watertight),
        "winding_consistent": bool(mesh.is_winding_consistent),
        "degenerate_faces": int((areas <= 1e-12).sum()),
        "connected_components": int(len(mesh.split(only_watertight=False))),
        "extents": extents,
    }


def _inertial(mesh, spec, warnings):
    """Mass and inertia, labelled with where each number came from."""
    if mesh.is_watertight and mesh.is_winding_consistent and mesh.volume > 0:
        solid, volume_source = mesh, "mesh (watertight)"
    else:
        solid, volume_source = mesh.convex_hull, "convex hull (mesh not watertight)"
        warnings.append("volume from convex hull: overestimates concave shapes")
    volume = float(solid.volume)
    if spec.mass_kg is not None:
        mass, mass_source = spec.mass_kg, "stated mass_kg"
    else:
        mass = spec.density_kg_m3 * volume
        mass_source = f"assumed density {spec.density_kg_m3} kg/m^3 x volume"
    # trimesh inertia assumes density 1 -> rescale to the chosen mass.
    inertia = solid.moment_inertia * (mass / volume)
    return {
        "mass_kg": mass,
        "mass_source": mass_source,
        "volume_m3": volume,
        "volume_source": volume_source,
        "center_of_mass": solid.center_mass.tolist(),
        "inertia": inertia.tolist(),
        "status": "ASSUMED - not measured; validate before trusting dynamics",
    }


def normalise(mesh, spec):
    """Rotate, scale and re-origin a mesh; derive collision and inertia."""
    spec.validate()
    warnings = []
    source_checks = mesh_checks(mesh)
    if not source_checks["finite_vertices"]:
        raise ValueError("mesh has non-finite vertices")
    rot = UP_ROTATIONS[spec.up_axis]
    rotated = mesh.vertices @ rot.T
    extents = rotated.max(axis=0) - rotated.min(axis=0)
    if spec.target_size_m is not None:
        along = extents[AXES[spec.target_axis]]
        if along <= 0:
            raise ValueError(f"mesh has zero extent along {spec.target_axis}")
        scale = spec.target_size_m / along
    else:
        scale = spec.source_unit_m
    scaled = rotated * scale
    lo, hi = scaled.min(axis=0), scaled.max(axis=0)
    translation = np.array([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])
    final = trimesh.Trimesh(scaled + translation, mesh.faces.copy(), process=False)
    final_extents = final.extents
    if final_extents.max() > MAX_EXTENT_M or final_extents.max() < MIN_EXTENT_M:
        raise ValueError(f"implausible size after scaling: {final_extents.tolist()} m")
    if final_extents.min() < MIN_EXTENT_M:
        warnings.append(f"thinnest extent {final_extents.min():.4f} m: may be a plane")

    collision_mesh = collision_box = None
    if spec.collision == "box":
        b_lo, b_hi = final.bounds
        collision_box = ((b_hi - b_lo).tolist(), ((b_hi + b_lo) / 2).tolist())
    elif spec.collision == "convex_hull":
        collision_mesh = final.convex_hull
    else:
        collision_mesh = final
        if len(final.faces) > spec.max_collision_tris:
            raise ValueError(
                f"{len(final.faces)} triangles exceeds the collision budget of "
                f"{spec.max_collision_tris}; use convex_hull/box, a lower-poly "
                "source (ProjectSim mesh.quality low/ultra_low) or raise the limit")
        if not source_checks["watertight"]:
            warnings.append("collision mesh is not watertight: fine for static "
                            "obstacles, unreliable for dynamic contact")
    if spec.collision == "convex_hull" and source_checks["connected_components"] > 1:
        warnings.append("convex hull fills gaps between parts: sensors still see "
                        "the visual mesh, but contact will not")

    inertial = None if spec.static else _inertial(final, spec, warnings)
    checks = {"source": source_checks, "normalised": mesh_checks(final)}
    return ImportResult(rot, float(scale), translation, final, collision_mesh,
                        collision_box, checks, inertial, warnings)
