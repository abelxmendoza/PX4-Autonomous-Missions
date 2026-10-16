// Pure, side-effect-free helpers for the bench instrument-panel replay page,
// shared between the page (loaded as a plain <script>, defining globals) and
// its test suite (imported as a CommonJS module) -- same pattern as lib.js
// and search-replay.js, so the tested code is the code the page runs.

function parseBenchCampaign(jsonText) {
  const data = JSON.parse(jsonText);
  if (!data || !data.report || !Array.isArray(data.samples)) {
    throw new Error('bench_campaign.json: missing report or samples');
  }
  if (data.samples.length === 0) throw new Error('bench_campaign.json: no samples');
  return data;
}

// Linear interpolation between the two samples bracketing `t`; fault_active
// and the overall fault-window membership are taken from the earlier sample
// (a string can't be interpolated, and "is a fault active right now" should
// not blend across a window boundary).
function sampleAt(samples, t) {
  const n = samples.length;
  const t0 = samples[0].t, tEnd = samples[n - 1].t;
  const clamped = Math.max(t0, Math.min(tEnd, t));
  const dt = n > 1 ? (tEnd - t0) / (n - 1) : 0;
  const idx = dt > 0 ? Math.floor((clamped - t0) / dt) : 0;
  const i = Math.max(0, Math.min(n - 2, idx));
  const a = samples[i], b = samples[Math.min(n - 1, i + 1)];
  const span = b.t - a.t;
  const frac = span > 0 ? (clamped - a.t) / span : 0;
  const lerp = (ka) => a[ka] + (b[ka] - a[ka]) * frac;
  return {
    t: clamped,
    command_pct: lerp('command_pct'),
    rpm: lerp('rpm'),
    thrust_n: lerp('thrust_n'),
    load_cell_n: lerp('load_cell_n'),
    voltage_v: lerp('voltage_v'),
    temp_c: lerp('temp_c'),
    fault_active: a.fault_active,
  };
}

// Requirement id -> whether it is "live-failing" at time t, i.e. the fault
// window its detail is about is currently open. Used to light a status LED
// only while the relevant fault is actually injected, not for the whole replay.
function faultWindowsFor(samples) {
  const windows = [];
  let open = null;
  for (const s of samples) {
    if (s.fault_active && (!open || open.name !== s.fault_active)) {
      if (open) windows.push(open);
      open = { name: s.fault_active, start_s: s.t, end_s: s.t };
    } else if (s.fault_active && open) {
      open.end_s = s.t;
    } else if (!s.fault_active && open) {
      windows.push(open);
      open = null;
    }
  }
  if (open) windows.push(open);
  return windows;
}

if (typeof module !== 'undefined' && module.exports) {
  module.exports = { parseBenchCampaign, sampleAt, faultWindowsFor };
}
