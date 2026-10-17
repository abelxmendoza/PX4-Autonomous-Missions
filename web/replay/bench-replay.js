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

// Plain-English names for the automatic checks, for people who don't read requirement IDs.
const BENCH_PLAIN = {
  'REQ-BENCH-001': 'The motor never pushes the wrong way (thrust is never negative)',
  'REQ-BENCH-002': 'When the thrust sensor fails, the failure shows up in the data instead of hiding',
  'REQ-BENCH-003': 'The battery never sags below the safe 9 V level',
  'REQ-BENCH-004': 'No measurements went missing during the test',
  'REQ-BENCH-005': 'A frozen temperature sensor is caught within 1 second, with no false alarms',
};

// Throttle steps in the recording: [{t, from, to}] wherever the commanded throttle changes.
function throttleSteps(samples) {
  const steps = [];
  for (let i = 1; i < samples.length; i++) {
    if (samples[i].command_pct !== samples[i - 1].command_pct) {
      steps.push({t: samples[i].t, from: samples[i - 1].command_pct, to: samples[i].command_pct});
    }
  }
  return steps;
}

// What is happening at time t, in words: {title, text, tone}. Injected faults take priority
// over throttle changes; everything is derived from the recording and its report.
function benchStory(data, t) {
  const samples = data.samples;
  const s = sampleAt(samples, t);
  const end = samples[samples.length - 1].t;
  const stuck = (data.report.events || []).find(e => e.event === 'stuck_sensor_flagged');
  if (t >= end - 1e-6) {
    const reqs = data.report.requirements;
    const passed = reqs.filter(r => r.passed).length;
    return {tone: passed === reqs.length ? 'good' : 'bad', title: 'Test complete',
            text: `${passed} of ${reqs.length} automatic checks passed. Every check is graded by software from the recorded data, the same way on every run.`};
  }
  if (s.fault_active === 'load_cell_dropout') {
    return {tone: 'fault', title: 'Sensor failure (injected on purpose)',
            text: 'We cut the thrust sensor’s signal. Its reading drops to zero, but the motor is still spinning. A good test must recognise that the sensor broke, not conclude the motor stopped.'};
  }
  if (s.fault_active === 'frozen_temp_sensor') {
    if (stuck && t >= stuck.t) {
      return {tone: 'good', title: `Frozen sensor caught at ${stuck.t.toFixed(1)} s`,
              text: 'The automatic check noticed the temperature reading has not changed at all for half a second. Real sensors always jitter a little, so a perfectly still reading means the sensor is stuck.'};
    }
    return {tone: 'fault', title: 'Temperature sensor frozen (injected on purpose)',
            text: 'The temperature reading is stuck on one number, while the motor may really be getting hotter. Watch for the automatic check to catch it.'};
  }
  const steps = throttleSteps(samples).filter(st => st.t <= t + 1e-6);
  if (!steps.length) {
    return {tone: 'calm', title: 'Ready',
            text: 'The motor is clamped to the test bench and not spinning yet. The automated test is about to start.'};
  }
  const last = steps[steps.length - 1];
  const maxCmd = Math.max(...samples.map(x => x.command_pct));
  if (last.to === 0) {
    return {tone: 'calm', title: 'Motor off', text: 'The test throttles back to zero and the measured thrust falls with it.'};
  }
  if (last.to === maxCmd) {
    return {tone: 'calm', title: `Full power test: ${last.to.toFixed(0)}% throttle`,
            text: 'Maximum load. The battery voltage dips under the strain and the motor slowly heats up, just as a real motor and battery would.'};
  }
  const verb = last.to > last.from ? 'up' : 'down';
  return {tone: 'calm', title: `Throttle ${verb} to ${last.to.toFixed(0)}%`,
          text: verb === 'up'
            ? 'The test asks the motor for more power. The measured thrust (green) trails the request (orange) for a moment: a motor needs time to spin up.'
            : 'The test asks for less power, and the thrust settles to a lower level.'};
}

if (typeof module !== 'undefined' && module.exports) {
  module.exports = { parseBenchCampaign, sampleAt, faultWindowsFor, throttleSteps, benchStory, BENCH_PLAIN };
}
