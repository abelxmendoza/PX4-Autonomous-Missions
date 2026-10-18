# RealmForge XR → PX4 / Gazebo environment adapter

Initial experimental integration. RealmForge remains the world editor; PX4 and Gazebo remain responsible for autonomous flight and physics.

## Export a world

In RealmForge XR, export a `realmforge-world` version 1 JSON file. Then run:

```bash
python3 tools/realmforge_to_sdf.py ~/Downloads/my-world.json /tmp/realmforge.sdf
```

The output is a standalone SDF world with a ground box and static primitive collision boxes. It is **not yet** a drop-in PX4 SITL launch target: add the required PX4/Gazebo vehicle spawn and world integration in a later milestone.

## Coordinate mapping

RealmForge uses x-right, y-up, z-toward-viewer, with object position at the base center and size [width, height, depth]. The adapter maps to Gazebo ENU x, y, z using `(x, -z, y)`, negative yaw, and box sizes `(width, depth, height)`. This is a deliberate mapping convention, not a measured real-world georeference.

## Current scope

- Exports static box collision and visual geometry for objects marked solid, default-blocking walls/buildings/pallets/racks, and raised platforms.
- Does not export decorative objects, mesh scans, material textures, robot routes, traffic zones, sensor models, georeferencing, or safety claims.
- No continuous collision validation or flight testing has been performed.
- Next: fixture tests for transforms/obstacle selection; Gazebo loading smoke test; hook a PX4 SITL launch script; then connect a browser replay overlay to recorded telemetry.

Do not interpret a successful conversion as a validated simulation or safe physical flight environment.
