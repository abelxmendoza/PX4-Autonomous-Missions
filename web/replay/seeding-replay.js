// Pure replay logic for the Cloud Seeding demo: no DOM, no THREE. The page draws what
// these functions compute; vitest checks them against the recorded flights.
//
// A trace (data/seeding/<scenario>.json, written by tools/integrated_seeding_sitl.py --trace)
// holds PX4's own pose estimate at 10 Hz tagged with the orchestrator's state, plus the
// orchestrator's evidence on the same mission clock: state transitions, the release-gate
// verdicts, release samples and marker detections. Marker ground truth comes separately
// (data/search_targets.json) and is used for display and scoring only.

const SEEDING_PHASES = [
  {id: 'takeoff', label: 'Takeoff', color: '#9db1ad'},
  {id: 'transit_to_release', label: 'Transit', color: '#ffb547'},
  {id: 'hold_for_release', label: 'Release check', color: '#ffd27a'},
  {id: 'dispensing', label: 'Dispensing', color: '#69f7cd'},
  {id: 'return_to_recovery', label: 'Return', color: '#7aa2ff'},
  {id: 'precision_landing', label: 'Precision landing', color: '#ff5da2'},
  {id: 'end', label: 'Landed', color: '#ffffff'},
];
const TERMINAL = new Set(['landed', 'aborted', 'failed']);
const GATE_LABELS = {altitude: 'Altitude band', weather: 'Weather', airspace: 'Airspace', traffic: 'Traffic', comms: 'Comms / clearance'};

function parseSeedingTrace(text) {
  const d = JSON.parse(text);
  if (d.schema !== 1 || d.kind !== 'seeding_sortie') throw new Error('Not a seeding trace');
  if (!Array.isArray(d.frames) || d.frames.length < 2) throw new Error('Seeding trace has no flight path');
  let prev = -Infinity;
  for (const f of d.frames) {
    if (![f.t, f.n, f.e, f.d, f.yaw_deg].every(Number.isFinite)) throw new Error('Invalid pose sample');
    if (f.t < prev) throw new Error('Seeding timeline goes backwards');
    prev = f.t;
  }
  for (const key of ['states', 'detections']) if (!Array.isArray(d[key])) throw new Error(`Seeding trace lacks ${key}`);
  if (!d.gates || !Object.keys(GATE_LABELS).every(g => d.gates[g])) throw new Error('Seeding trace lacks gate verdicts');
  return d;
}

function _angleLerp(a, b, k) {
  const diff = ((b - a + 540) % 360) - 180;
  let v = a + diff * k;
  if (v > 180) v -= 360;
  if (v <= -180) v += 360;
  return v;
}

function seedingPoseAt(frames, t) {
  if (t <= frames[0].t) return {...frames[0], t};
  const last = frames[frames.length - 1];
  if (t >= last.t) return {...last, t};
  let lo = 0, hi = frames.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (frames[mid].t <= t) lo = mid; else hi = mid;
  }
  const a = frames[lo], b = frames[hi];
  const k = b.t > a.t ? (t - a.t) / (b.t - a.t) : 0;
  return {...a, t, n: a.n + (b.n - a.n) * k, e: a.e + (b.e - a.e) * k, d: a.d + (b.d - a.d) * k,
          yaw_deg: _angleLerp(a.yaw_deg, b.yaw_deg, k)};
}

// Orchestrator state at time t (from its own transition log). Before the mission clock
// starts (t < 0, PX4's takeoff) the state is "takeoff".
function stateAt(trace, t) {
  let state = 'takeoff', reason = '';
  for (const s of trace.states) {
    if (s.t > t) break;
    if (s.to === 'preflight') continue;   // instantaneous; the sortie shows as takeoff until transit
    state = s.to; reason = s.reason;
  }
  return {state, reason};
}

function phaseIndex(state) {
  if (TERMINAL.has(state)) return SEEDING_PHASES.length - 1;
  const i = SEEDING_PHASES.findIndex(p => p.id === state);
  return i < 0 ? 0 : i;
}

function phaseColor(state) {
  return SEEDING_PHASES[phaseIndex(state)].color;
}

// When the release decision was made: the vehicle left hold_for_release (to dispensing if
// every gate allowed it, otherwise straight to the return).
function releaseDecisionTime(trace) {
  const left = trace.states.find(s => s.from === 'hold_for_release');
  return left ? left.t : null;
}

// Gate panel at time t: "pending" until the release decision, then each gate's verdict.
function gatesAt(trace, t) {
  const decided = releaseDecisionTime(trace);
  return Object.keys(GATE_LABELS).map(id => {
    const g = trace.gates[id];
    const status = decided === null || t < decided ? 'pending' : (g.authorized ? 'pass' : 'fail');
    return {id, label: GATE_LABELS[id], status, reason: status === 'fail' ? g.reason : ''};
  });
}

// Seed released by time t: the recorded total, spread over the release samples.
function releasedAt(trace, t) {
  const samples = trace.release.samples;
  if (!samples.length) return 0;
  const done = samples.filter(s => s.t <= t).length;
  return trace.release.amount_kg * done / samples.length;
}

function detectionsUntil(trace, t) {
  return trace.detections.filter(d => d.t <= t);
}

// The landing camera's ground footprint (level-drone approximation, as in the search replay).
function seedingFootprint(pose, camera) {
  const h = -pose.d;
  if (!camera || !(h > 0)) return [];
  const halfW = h * Math.tan(camera.hfov_rad / 2);
  const halfL = halfW * camera.height_px / camera.width_px;
  const y = pose.yaw_deg * Math.PI / 180;
  const fwd = {n: Math.cos(y), e: Math.sin(y)}, right = {n: -Math.sin(y), e: Math.cos(y)};
  const at = (f, r) => ({n: pose.n + fwd.n * f + right.n * r, e: pose.e + fwd.e * f + right.e * r});
  return [at(halfL, -halfW), at(halfL, halfW), at(-halfL, halfW), at(-halfL, -halfW)];
}

// Touchdown against the marker the drone locked onto (truth from the world file).
function touchdownScore(trace, truth) {
  const td = trace.touchdown;
  const target = truth && truth.targets.find(x => x.id === trace.marker_id);
  if (!td || !target) return null;
  return {markerId: trace.marker_id, errorM: Math.hypot(td.north_m - target.north, td.east_m - target.east),
          markerN: target.north, markerE: target.east};
}

// One-line summary of how the sortie ended, in words.
function outcomeText(trace) {
  const o = trace.outcomes || {};
  const release = o.release_result === 'authorized' ? 'Release authorized' : `Release denied: ${o.release_reason_code || 'unknown'}`;
  const recovery = o.recovery_result === 'landed' ? 'landed on the pad' : `recovery failed: ${o.recovery_reason_code || trace.mission_result}`;
  return `${release}; ${recovery}.`;
}

function seedingNedToScene(n, e, d) {
  return {x: e, y: -d, z: -n};
}

if (typeof module !== 'undefined') {
  module.exports = {SEEDING_PHASES, GATE_LABELS, parseSeedingTrace, seedingPoseAt, stateAt, phaseIndex, phaseColor,
                    releaseDecisionTime, gatesAt, releasedAt, detectionsUntil, seedingFootprint, touchdownScore,
                    outcomeText, seedingNedToScene};
}
