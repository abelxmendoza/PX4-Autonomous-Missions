"""Mesh -> Gazebo Harmonic asset pipeline.

Modules:
  mesh_import   normalise any mesh (axis, scale, origin), check it, derive
                collision geometry and assumed mass/inertia
  gz_model      write a Gazebo model directory plus a provenance manifest
  world_builder seeded, reproducible placement of models into a PX4 world
  drop_test     load a model in headless Gazebo and drop a probe ball on it
  projectsim    OPTIONAL ProjectSim API backend with a credit ledger and a
                spend guard; nothing else in this package imports it

The pipeline works on local meshes with no network access. ProjectSim is
one possible mesh source, not a dependency.
"""
