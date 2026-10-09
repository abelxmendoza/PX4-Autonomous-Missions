// Pure replay logic for the SLAM & Costmap page: no DOM. The page draws what these
// functions compute; vitest checks them against the recorded flight.
//
// The trace (data/slam_trace.json, a copy of evidence/slam/) comes from tools/slam_flight.py:
// a PX4/Gazebo flight where every real LiDAR scan went through 2-D SLAM. Poses are NED
// metres/radians (north, east, yaw). The map and costmap are in SLAM's own map frame;
// report.runs[*].map_frame_offset is the rigid transform onto Gazebo's frame, fitted after
// the flight for scoring only.

const LETHAL = 254, INSCRIBED = 253, NO_INFO = 255;

function parseSlamTrace(text) {
  const d = JSON.parse(text);
  const g = d.grid;
  if (!g || ![g.resolution, g.north_min, g.east_min, g.rows, g.cols].every(Number.isFinite)) throw new Error('Invalid SLAM grid');
  if (!d.samples || !Array.isArray(d.samples.drifting_odometry) || d.samples.drifting_odometry.length < 2) throw new Error('Invalid SLAM samples');
  for (const run of Object.values(d.samples)) {
    let prev = -Infinity;
    for (const s of run) {
      if (s.length !== 10 || !s.every(Number.isFinite)) throw new Error('Invalid SLAM sample');
      if (s[0] < prev) throw new Error('Invalid SLAM timeline');
      prev = s[0];
    }
  }
  const c = d.costmap;
  if (!c || c.rows !== g.rows || c.cols !== g.cols || c.costs_rle.length % 2) throw new Error('Costmap does not match the map grid');
  return d;
}

function decodeRle(rle, size) {
  const out = new Uint8Array(size);
  let k = 0;
  for (let i = 0; i < rle.length; i += 2) {
    out.fill(rle[i + 1], k, k + rle[i]);
    k += rle[i];
  }
  if (k !== size) throw new Error(`RLE covers ${k} cells, expected ${size}`);
  return out;
}

// Same bands and colours as costmap_markers.py (the Gazebo display).
function costBand(cost) {
  if (cost === NO_INFO || cost === 0) return null;
  if (cost === LETHAL) return 'lethal';
  if (cost === INSCRIBED) return 'inscribed';
  return cost >= 128 ? 'high' : 'low';
}

// Index of the last sample at or before t (0 if t precedes the first).
function sampleIndexAt(samples, t) {
  let lo = 0, hi = samples.length - 1;
  if (t <= samples[0][0]) return 0;
  if (t >= samples[hi][0]) return hi;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (samples[mid][0] <= t) lo = mid; else hi = mid;
  }
  return lo;
}

// Map cells visible at time t: first seen by then and still occupied in the final map
// (cells that were later cleared as free space are not drawn).
function mapCellsAt(trace, costs, t) {
  return trace.map_cells.filter(([idx, seen]) => seen <= t && costs[idx] === LETHAL).map(([idx]) => idx);
}

function cellCentre(grid, idx) {
  const i = Math.floor(idx / grid.cols), j = idx % grid.cols;
  return [grid.north_min + (i + 0.5) * grid.resolution, grid.east_min + (j + 0.5) * grid.resolution];
}

// compose((dn, de, dyaw), (n, e)): a SLAM-frame point in Gazebo's frame.
function applyFrame(offset, n, e) {
  const a = offset.yaw_deg * Math.PI / 180, c = Math.cos(a), s = Math.sin(a);
  return [offset.north_m + c * n - s * e, offset.east_m + s * n + c * e];
}

// Columns: t, truth n/e/yaw, odometry n/e/yaw, slam n/e/yaw.
function errorsAt(sample, offset) {
  const [, tn, te, , on, oe, , sn, se] = sample;
  const [an, ae] = offset ? applyFrame(offset, sn, se) : [sn, se];
  return {odometry: Math.hypot(on - tn, oe - te), slam: Math.hypot(an - tn, ae - te)};
}

if (typeof module !== 'undefined') {
  module.exports = {LETHAL, INSCRIBED, NO_INFO, parseSlamTrace, decodeRle, costBand, sampleIndexAt,
                    mapCellsAt, cellCentre, applyFrame, errorsAt};
}
