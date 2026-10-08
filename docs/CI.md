# Continuous integration

CI is [`.github/workflows/ci.yml`](../.github/workflows/ci.yml). It runs on every push and pull
request and does **not** fly anything: no Gazebo, no PX4. Simulated flights are slow, need a GPU/EGL
setup and are not deterministic enough to gate a merge on a shared runner. CI instead runs what can be made
deterministic: unit tests, recorded-flight replay, seeded fault-injection scenarios, the requirement
registry, and the C++ tests.

Every CI step is a `make` target, so the same commands run locally ([below](#reproduce-it-locally)).

## Jobs

| Job | Runs | Blocking | Produces |
| --- | --- | --- | --- |
| `unit` | `make test-fast` -- ROS-free pytest, no simulated flights | yes | `artifacts/junit/unit.xml` |
| `validation` | `make test-fast test-integration evidence faults validate compare-selfcheck` | yes | JUnit, fault evidence (JSON, Markdown, per-fault CSV), validation report (JSON + Markdown), compare output; the Markdown report is also written to the job summary |
| `cpp` | `make cpp-test` -- CMake configure (warnings as errors), build, CTest | yes | `artifacts/junit/cpp.xml` |
| `ros2` | `colcon build` + full pytest inside `ros:humble` | **no** (`continue-on-error`) | `artifacts/junit/ros.xml` |

### What makes the build fail

- any failing pytest or CTest case (fast and integration tests are separated by the `integration` marker);
- `make evidence`: a curated recording no longer produces its pinned verdict or hash
  ([`scripts/verify_evidence.py`](../scripts/verify_evidence.py)), the C++/Python golden MAVLink vectors
  are stale, or the exported Gazebo world differs from its source;
- `make faults`: a fault scenario's expectation fails, unless the fault is declared `known_gap`;
- `make validate`: **a requirement fails** -- see below.

#### Requirement gate

[`tools/validation_report.py`](../tools/validation_report.py) evaluates
[`requirements/registry.yaml`](../requirements/registry.yaml) and exits non-zero when:

1. a requirement is `FAIL` and is not declared `known_open`;
2. a requirement is declared `known_open` but no longer fails (a stale marker -- remove it);
3. a requirement is `PARTIAL`/`NOT_RUN` for a reason other than an `external` (hardware / live SITL)
   verification that was not run -- i.e. evidence that should exist is missing.

Three requirements are currently `known_open` and fail: REQ-EST-001 (VO drift), REQ-EST-002 (VO
availability) and REQ-EST-006 (frozen-IMU detection). They fail because the measurements or tests say so,
they are visible at the top of the report, and the gate keeps them from being forgotten or quietly passing.
**The build is green with three failing requirements; that is a statement about the gate, not about the
requirements.** Read [`validation_report.md`](#artifacts) rather than the badge.

### Artifacts

Uploaded on every run (also when a step fails): `junit-unit`, `junit-cpp`, `junit-ros2`, and
`validation-artifacts` containing `artifacts/junit/*.xml`, `artifacts/faults/*.json|md` and per-fault
time-series CSV, `artifacts/validation/validation_report.{json,md}`, and `artifacts/compare/K_vs_L.json`.
The validation report maps Requirement -> Test -> Evidence -> Result and lists what was *not* run.

### What CI does not establish

- It does not run PX4, Gazebo, or any flight. Flight results come from curated recordings made earlier on a
  workstation (see [engineering results](engineering_results.md)); CI re-checks those recordings, not the
  simulator.
- The `ros2` job has **never been run on a GitHub runner** by the author of this change (it clones `px4_msgs` from the network, pinned to commit
  `392e831`). It is non-blocking so a first-run infrastructure problem cannot hold up
  merges; make it blocking (delete `continue-on-error`) after its first green run. The equivalent local
  commands (`colcon build`, full `pytest` with ROS sourced) have been run: 450 tests passed.
- No physical hardware is involved. REQ-COMMS-006 (physical I2C/SPI) and REQ-HIL-004 (real flight
  controller) are `NOT_RUN`.
- The workflow file itself was validated as YAML and its `make` targets were run end to end locally in a
  clean ROS-free virtualenv; it has not been exercised on GitHub's infrastructure from here.

## Reproduce it locally

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements-ci.txt          # ROS is not needed for the blocking jobs
make ci                                      # everything the blocking jobs run, in order
```

Or individually: `make test-fast`, `make test-integration`, `make evidence`, `make faults`,
`make validate`, `make compare-selfcheck`, `make cpp-test`. Outputs land in `artifacts/` (git-ignored);
`make clean` removes them and the C++ build directory.

Do not mix a sourced ROS shell with a virtualenv: ROS's pytest plugins then import into an interpreter without
`lark` and `make test-fast` fails during collection (seen locally). Use a fresh shell for the ROS-free run.

With ROS (also runs the node tests that are otherwise skipped):

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -m pytest -q
```

The C++ steps by hand:

```bash
cmake -S cpp -B build/cpp -DCMAKE_BUILD_TYPE=Release -DPX4V_WERROR=ON   # configure
cmake --build build/cpp --parallel                                        # build
ctest --test-dir build/cpp --output-on-failure                            # test
cmake --build build/cpp --target clean                                    # clean (or: rm -rf build/cpp)
```

### How the C++ relates to the ROS 2 build

`px4_offboard` is an `ament_python` package built by colcon; that is unchanged. The C++ lives in
[`cpp/`](../cpp/CMakeLists.txt), a plain CMake project that builds independently:

- `px4v_mavlink_frame` -- a dependency-free MAVLink v2 codec (static library, `px4v::mavlink_frame` alias);
- `test_mavlink_frame` -- checks it against `cpp/test/golden_frames.txt`, vectors generated by the Python
  codec (itself checked byte-for-byte against pymavlink) and registered with CTest;
- `gz_cam_sub` -- the Gazebo camera wake-up subscriber, compiled from the existing `scripts/gz_cam_sub.cpp`
  when `gz-transport13` and `gz-msgs10` are found (the launch file still builds it on demand with `g++`).

### Repeatability

All randomness in the fault rigs and the unit tests is seeded; the same seed gives the same result on every
machine (tests assert this). Live-flight results are different: identical-configuration flights K and L ended
at 9.3% and 13.2% drift. That run-to-run spread is why `tools/compare_runs.py` reports ranges and refuses to
call a single-run difference an established regression, and why REQ-EST-001/002 remain open instead of being
re-tuned until a run passes.
