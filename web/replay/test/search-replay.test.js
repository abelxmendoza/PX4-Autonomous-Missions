import {describe, it, expect} from 'vitest';
import {createRequire} from 'node:module';
const require = createRequire(import.meta.url);
const S = require('../search-replay.js');
const trace = require('../data/search_trace.json');
const truth = require('../data/search_targets.json');

describe('search flight replay', () => {
  const data = S.parseSearchTrace(JSON.stringify(trace));

  it('parses the recorded flight and rejects broken ones', () => {
    expect(data.frames.length).toBeGreaterThan(500);
    const bad = JSON.parse(JSON.stringify(trace));
    bad.frames[10].t = bad.frames[9].t - 1;
    expect(() => S.parseSearchTrace(JSON.stringify(bad))).toThrow(/timeline/);
    expect(() => S.parseSearchTrace(JSON.stringify({schema: 2}))).toThrow();
  });

  it('interpolates the pose between samples, including yaw across +-180', () => {
    const frames = [{t: 0, n: 0, e: 0, d: -8, yaw_deg: 170}, {t: 1, n: 2, e: 4, d: -8, yaw_deg: -170}];
    const p = S.poseAt(frames, 0.5);
    expect([p.n, p.e, p.d]).toEqual([1, 2, -8]);
    expect(Math.abs(p.yaw_deg)).toBeCloseTo(180, 6); // through 180, not through 0
    expect(S.poseAt(frames, -5).n).toBe(0);          // clamped at the ends
    expect(S.poseAt(frames, 99).e).toBe(4);
  });

  it('camera footprint at 8 m matches the camera model: about 19 x 14 m', () => {
    const fp = S.footprint({n: 10, e: 0, d: -8, yaw_deg: 0}, data.camera);
    const width = Math.hypot(fp[1].n - fp[0].n, fp[1].e - fp[0].e);   // across track (right)
    const length = Math.hypot(fp[3].n - fp[0].n, fp[3].e - fp[0].e);  // along track (forward)
    expect(width).toBeCloseTo(2 * 8 * Math.tan(1.74 / 2), 6);
    expect(length).toBeGreaterThan(13); expect(length).toBeLessThan(15);
    // facing north, the far edge of the footprint is north of the drone
    expect(Math.max(...fp.map(c => c.n))).toBeGreaterThan(10);
  });

  it('a target counts as found only from its confirmation time, at the median of its sightings so far', () => {
    const first = [...data.confirmations].sort((a, b) => a.t - b.t)[0];
    expect(S.foundAt(data, first.t - 0.01).size).toBe(0);
    const found = S.foundAt(data, first.t);
    expect([...found.keys()]).toEqual([first.id]);
    expect(Number.isFinite(found.get(first.id).n)).toBe(true);
  });

  it('by the end every target is found, matching the recorded report', () => {
    const end = S.foundAt(data, data.frames.at(-1).t);
    expect([...end.keys()].sort((a, b) => a - b)).toEqual(truth.targets.map(t => t.id).sort((a, b) => a - b));
    for (const t of data.final_targets) {
      expect(end.get(t.id).n).toBeCloseTo(t.north, 1);
      expect(end.get(t.id).e).toBeCloseTo(t.east, 1);
    }
  });

  it('scores against the truth file with the same rule as the Python scorer', () => {
    const s = S.scoreFound(S.foundAt(data, data.frames.at(-1).t), truth);
    expect(s.found).toBe(8); expect(s.falseIds).toEqual([]); expect(s.missed).toEqual([]);
    expect(s.maxError).toBeLessThan(1.0);
  });

  it('the trail is the flown path up to now, never ahead of the drone', () => {
    const t = 30;
    const trail = S.trailUntil(data.frames, t);
    expect(trail.length).toBeGreaterThan(10);
    expect(trail.every(f => f.t <= t)).toBe(true);
  });

  it('maps NED to the scene like the swarm replay: x=east, y=up, z=-north', () => {
    expect(S.nedToScene(10, 3, -8)).toEqual({x: 3, y: 8, z: -10});
  });
});
