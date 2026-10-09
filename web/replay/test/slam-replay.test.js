// The SLAM & Costmap page's replay logic, checked against the committed flight.
import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { createRequire } from 'node:module';

const require = createRequire(import.meta.url);
const R = require('../slam-replay.js');
const SITE = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const text = readFileSync(join(SITE, 'data/slam_trace.json'), 'utf8');
const trace = R.parseSlamTrace(text);
const g = trace.grid;
const costs = R.decodeRle(trace.costmap.costs_rle, g.rows * g.cols);
const drift = trace.samples.drifting_odometry;
const report = trace.report;

describe('slam replay', () => {
  it('the site copy of the trace is byte-identical to the committed evidence', () => {
    expect(text).toBe(readFileSync(join(SITE, '../../evidence/slam/obstacle_course_slam_trace.json'), 'utf8'));
    // The trace embeds the same report as the evidence file (compared as values: Python and
    // JavaScript print floats like 2.0 differently).
    expect(report).toEqual(JSON.parse(readFileSync(join(SITE, '../../evidence/slam/obstacle_course_slam_report.json'), 'utf8')));
  });

  it('decodes the costmap and refuses a run-length that does not cover the grid', () => {
    expect(costs.length).toBe(g.rows * g.cols);
    expect(() => R.decodeRle([5, 0], 6)).toThrow();
    expect(Array.from(R.decodeRle([2, 7, 3, 254], 5))).toEqual([7, 7, 254, 254, 254]);
  });

  it('bands costs the same way as the Gazebo display', () => {
    expect([0, 1, 127, 128, 252, 253, 254, 255].map(R.costBand))
      .toEqual([null, 'low', 'low', 'high', 'high', 'inscribed', 'lethal', null]);
  });

  it('the map grows over the flight and ends on the final occupied cells', () => {
    const t0 = drift[0][0], t1 = drift[drift.length - 1][0];
    const early = R.mapCellsAt(trace, costs, t0 + 5).length, late = R.mapCellsAt(trace, costs, t1 + 1).length;
    expect(early).toBeGreaterThan(0);
    expect(late).toBeGreaterThan(early * 2);
    for (const idx of R.mapCellsAt(trace, costs, t1 + 1)) expect(costs[idx]).toBe(R.LETHAL);
  });

  it('finds the sample at a time by binary search', () => {
    const mid = drift[200][0];
    expect(R.sampleIndexAt(drift, mid)).toBe(200);
    expect(R.sampleIndexAt(drift, mid + 1e-6)).toBe(200);
    expect(R.sampleIndexAt(drift, -1)).toBe(0);
    expect(R.sampleIndexAt(drift, 1e12)).toBe(drift.length - 1);
  });

  it('recomputes the reported errors from the samples', () => {
    const run = report.runs.drifting_odometry, offset = run.map_frame_offset;
    const errs = drift.map(s => R.errorsAt(s, offset));
    const mean = errs.reduce((a, e) => a + e.slam, 0) / errs.length;
    expect(mean).toBeCloseTo(run.slam_ate_aligned_mean_m, 2);
    expect(Math.max(...errs.map(e => e.slam))).toBeCloseTo(run.slam_ate_aligned_max_m, 2);
    expect(R.errorsAt(drift[drift.length - 1]).odometry).toBeCloseTo(run.odometry_final_m, 2);
    expect(run.odometry_final_m).toBeGreaterThan(10);           // dead reckoning walked off...
    expect(run.slam_ate_aligned_mean_m).toBeLessThan(0.2);      // ...while SLAM tracked the path
  });

  it('applying the frame offset puts the map on the real obstacles', () => {
    const offset = report.runs.drifting_odometry.map_frame_offset;
    const boxes = trace.obstacles_truth;
    const near = idx => {
      const [n, e] = R.applyFrame(offset, ...R.cellCentre(g, idx));
      return boxes.some(b => Math.abs(n - b.north) <= b.size_north / 2 + 0.5 && Math.abs(e - b.east) <= b.size_east / 2 + 0.5);
    };
    for (const b of boxes) {
      const hit = R.mapCellsAt(trace, costs, Infinity).some(idx => {
        const [n, e] = R.applyFrame(offset, ...R.cellCentre(g, idx));
        return Math.abs(n - b.north) <= b.size_north / 2 + 0.5 && Math.abs(e - b.east) <= b.size_east / 2 + 0.5;
      });
      expect(hit, `box at ${b.north}, ${b.east}`).toBe(true);
    }
    expect(R.mapCellsAt(trace, costs, Infinity).filter(near).length).toBeGreaterThan(200);
  });

  it('rejects a malformed trace', () => {
    expect(() => R.parseSlamTrace('{}')).toThrow();
    const bad = JSON.parse(text);
    bad.samples.drifting_odometry[3] = [1, 2];
    expect(() => R.parseSlamTrace(JSON.stringify(bad))).toThrow();
  });
});
