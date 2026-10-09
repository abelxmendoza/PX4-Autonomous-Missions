// Pure replay logic for the Search & Find demo: no DOM, no THREE. The page draws what
// these functions compute; vitest checks them against the recorded flight.
//
// The trace holds only what the drone knew (PX4's pose estimate, geolocated sightings,
// confirmation times). Ground truth comes separately from data/search_targets.json and is
// used for display and scoring only.

function parseSearchTrace(text) {
  const data = JSON.parse(text);
  if (data.schema !== 1 || !Array.isArray(data.frames) || data.frames.length < 2) throw new Error('Invalid search trace');
  let prev = -Infinity;
  for (const f of data.frames) {
    if (![f.t, f.n, f.e, f.d, f.yaw_deg].every(Number.isFinite)) throw new Error('Invalid pose sample');
    if (f.t < prev) throw new Error('Invalid search timeline');
    prev = f.t;
  }
  for (const s of data.sightings || []) if (!Number.isInteger(s.id) || ![s.t, s.n, s.e].every(Number.isFinite)) throw new Error('Invalid sighting');
  for (const c of data.confirmations || []) if (!Number.isInteger(c.id) || !Number.isFinite(c.t)) throw new Error('Invalid confirmation');
  data.sightings = data.sightings || [];
  data.confirmations = data.confirmations || [];
  return data;
}

function _angleLerp(a, b, k) {
  const diff = ((b - a + 540) % 360) - 180;  // shortest way round
  let v = a + diff * k;
  if (v > 180) v -= 360;
  if (v <= -180) v += 360;
  return v;
}

function poseAt(frames, t) {
  if (t <= frames[0].t) return {...frames[0]};
  const last = frames[frames.length - 1];
  if (t >= last.t) return {...last};
  let lo = 0, hi = frames.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (frames[mid].t <= t) lo = mid; else hi = mid;
  }
  const a = frames[lo], b = frames[hi];
  const k = b.t > a.t ? (t - a.t) / (b.t - a.t) : 0;
  return {t, n: a.n + (b.n - a.n) * k, e: a.e + (b.e - a.e) * k, d: a.d + (b.d - a.d) * k,
          yaw_deg: _angleLerp(a.yaw_deg, b.yaw_deg, k)};
}

// Ground rectangle seen by the downward camera, assuming the drone is level (the trace
// keeps yaw only). Corners in order: front-left, front-right, back-right, back-left (NED).
function footprint(pose, camera) {
  const h = -pose.d;
  if (!(h > 0)) return [];
  const halfW = h * Math.tan(camera.hfov_rad / 2);
  const halfL = halfW * camera.height_px / camera.width_px;
  const y = pose.yaw_deg * Math.PI / 180;
  const fwd = {n: Math.cos(y), e: Math.sin(y)}, right = {n: -Math.sin(y), e: Math.cos(y)};
  const at = (f, r) => ({n: pose.n + fwd.n * f + right.n * r, e: pose.e + fwd.e * f + right.e * r});
  return [at(halfL, -halfW), at(halfL, halfW), at(-halfL, halfW), at(-halfL, -halfW)];
}

function _median(xs) {
  const s = [...xs].sort((a, b) => a - b);
  const m = s.length >> 1;
  return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2;
}

// Targets confirmed by time t -> their position estimate at that time (median of the
// sightings up to t, the same rule TargetTracker uses).
function foundAt(data, t) {
  const out = new Map();
  for (const c of data.confirmations) {
    if (c.t > t) continue;
    const seen = data.sightings.filter(s => s.id === c.id && s.t <= t);
    if (!seen.length) continue;
    out.set(c.id, {n: _median(seen.map(s => s.n)), e: _median(seen.map(s => s.e)), sightings: seen.length, t: c.t});
  }
  return out;
}

function scoreFound(found, truth) {
  const byId = new Map(truth.targets.map(t => [t.id, t]));
  const errors = [];
  const falseIds = [];
  for (const [id, est] of found) {
    const t = byId.get(id);
    if (!t) { falseIds.push(id); continue; }
    errors.push(Math.hypot(est.n - t.north, est.e - t.east));
  }
  return {
    found: errors.length,
    total: byId.size,
    missed: [...byId.keys()].filter(id => !found.has(id)).sort((a, b) => a - b),
    falseIds: falseIds.sort((a, b) => a - b),
    meanError: errors.length ? errors.reduce((a, b) => a + b, 0) / errors.length : null,
    maxError: errors.length ? Math.max(...errors) : null,
  };
}

function trailUntil(frames, t) {
  return frames.filter(f => f.t <= t);
}

function nedToScene(n, e, d) {
  return {x: e, y: -d, z: -n};
}

if (typeof module !== 'undefined') {
  module.exports = {parseSearchTrace, poseAt, footprint, foundAt, scoreFound, trailUntil, nedToScene};
}
