# RealmForge XR → PX4 / Gazebo environment adapter

Initial experimental integration. RealmForge remains the world editor; PX4 and Gazebo remain responsible for autonomous flight and physics.

## Export a world

In RealmForge XR, export a `realmforge-world` version 1 JSON file. Then run:

```bash
python3 tools/realmforge_to_sdf.py ~/Downloads/my-world.json /tmp/realmforge.sdf
```

The output is a standalone SDF world with a ground box and static primitive collision boxes. The existing swarm runner accepts the SDF through `--world-file`; live PX4/Gazebo compatibility still requires testing.

## Coordinate mapping

RealmForge uses x-right, y-up, z-toward-viewer, with object position at the base center and size [width, height, depth]. The adapter maps to Gazebo ENU x, y, z using `(x, -z, y)`, same-sign yaw, and box sizes `(width, depth, height)`. This is a deliberate mapping convention, not a measured real-world georeference.

## Current scope

- Exports static box collision and visual geometry for objects marked solid, default-blocking walls/buildings/pallets/racks, and raised platforms.
- Does not export decorative objects, mesh scans, material textures, robot routes, traffic zones, sensor models, measured georeferencing, or safety claims. It includes fixed PX4-compatible spherical coordinates for simulation.
- No continuous collision validation or flight testing has been performed.
- Fixture tests are in the main test suite and adapter CI. The fixture passed a local headless Gazebo load; next is a PX4 mission with explicit spawn and route clearance checks.

Do not interpret a successful conversion as a validated simulation or safe physical flight environment.

## Run the custom world with PX4 SITL (Linux workstation)

After generating the SDF and installing the existing repository's PX4, Gazebo, ROS 2 and MAVLink dependencies:

```bash
python3 scripts/run_swarm_demo.py --world-file /tmp/realmforge.sdf --dropout
```

The runner validates the SDF XML and reads its world name to configure PX4 and the ROS visualization trail. The default world is unchanged when `--world-file` is omitted.

**Caution:** The existing two-drone scenario has fixed spawn coordinates and planned routes. A RealmForge world can place walls across those positions or routes. Successful launch does not mean the mission is collision-free; use a clear scene for the first live test. The October 20 consolidation loaded the fixture in Gazebo for 1,000 iterations. A PX4 mission in the imported scene remains unvalidated.

## Phone access while the laptop is off

The hosted browser replay can be viewed on a phone independently of the laptop, as long as its deployment remains online and the evidence is hosted with it. Live PX4 SITL execution requires an always-on machine or cloud runner with PX4, Gazebo and ROS 2 installed. A static Vercel site cannot execute Gazebo/PX4. For remote live sessions, use an authenticated backend, job queue, isolated simulator worker, and read-only telemetry streaming first; do not expose MAVLink or ROS ports publicly.
