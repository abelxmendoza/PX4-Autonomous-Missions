# One entry point for local runs and CI, so "works on my machine" and "works in CI"
# are the same commands. See docs/CI.md.
PY      ?= python3
ART     ?= artifacts
CPP_BLD ?= build/cpp

.PHONY: help test-fast test-integration evidence faults validate compare-selfcheck \
        cpp-configure cpp-build cpp-test cpp-clean ci clean

help:
	@echo "targets: test-fast test-integration evidence faults validate compare-selfcheck cpp-test ci clean"

# Millisecond-scale unit tests (no simulated flights). ROS tests skip without ROS.
test-fast:
	@mkdir -p $(ART)/junit
	$(PY) -m pytest -m "not integration" --junitxml=$(ART)/junit/unit.xml -q

# Simulated flights / full pipelines (seconds): shipped fault scenarios, registry evaluation.
test-integration:
	@mkdir -p $(ART)/junit
	$(PY) -m pytest -m integration --junitxml=$(ART)/junit/integration.xml -q

# Curated recordings still produce their pinned verdicts; shared vectors/world files are current.
evidence:
	$(PY) scripts/verify_evidence.py
	$(PY) tools/gen_golden_frames.py --check
	$(PY) tools/gen_search_world.py --check
	$(PY) scripts/export_gazebo_world.py --check

faults:
	$(PY) tools/run_fault_scenarios.py config/fault_scenarios/*.yaml --out $(ART)/faults

# Requirement registry -> JSON + Markdown. Fails if any requirement fails without being a declared known-open item.
validate:
	$(PY) tools/validation_report.py --junit $(ART)/junit/unit.xml $(ART)/junit/integration.xml --out $(ART)/validation

# The regression comparator must run end to end on the curated live flights.
compare-selfcheck:
	@mkdir -p $(ART)/compare
	$(PY) tools/compare_runs.py evidence/stereo_vo/flight_K.csv.gz evidence/stereo_vo/flight_L.csv.gz --json $(ART)/compare/K_vs_L.json

cpp-configure:
	cmake -S cpp -B $(CPP_BLD) -DCMAKE_BUILD_TYPE=Release -DPX4V_WERROR=ON

cpp-build: cpp-configure
	cmake --build $(CPP_BLD) --parallel

cpp-test: cpp-build
	@mkdir -p $(ART)/junit
	ctest --test-dir $(CPP_BLD) --output-on-failure --output-junit $(abspath $(ART))/junit/cpp.xml

cpp-clean:
	rm -rf $(CPP_BLD)

# Everything CI runs (minus the ROS container job), in CI's order.
ci: test-fast test-integration evidence faults validate compare-selfcheck cpp-test

clean: cpp-clean
	rm -rf $(ART)
