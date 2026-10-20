import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { createRequire } from 'node:module';

const require = createRequire(import.meta.url);
const G = require('../guide.js');
const searchReplay = require('../search-replay.js');
const seedingReplay = require('../seeding-replay.js');
const slamReplay = require('../slam-replay.js');
const SITE = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const read = name => readFileSync(join(SITE, name), 'utf8');

describe('guide beat selection', () => {
  const beats = [{at: 0, title: 'a'}, {at: 10, title: 'b'}, {at: 20, title: 'c'}];
  it('stays on the latest beat at or before the flight time', () => {
    expect(G.beatIndex(beats, -1)).toBe(-1);
    expect(G.beatIndex(beats, 0).valueOf()).toBe(0);
    expect(G.beatIndex(beats, 10)).toBe(1);
    expect(G.beatIndex(beats, 19.9)).toBe(1);
    expect(G.beatIndex(beats, 20)).toBe(2);
  });
});

describe('search guide', () => {
  const trace = searchReplay.parseSearchTrace(read('data/search_trace.json'));
  const beats = G.searchGuideBeats(trace);
  it('opens on the sweep and marks the first and last confirmation', () => {
    expect(beats[0].at).toBe(0);
    expect(beats[0].title).toMatch(/sweep/i);
    const first = [...trace.confirmations].sort((a, b) => a.t - b.t)[0];
    expect(beats.some(b => Math.abs(b.at - (first.t - trace.frames[0].t)) < 1e-6 && b.text.includes(`#${first.id}`))).toBe(true);
    expect(beats.some(b => b.title === `All ${trace.confirmations.length} targets are confirmed`)).toBe(true);
    for (let i = 1; i < beats.length; i++) expect(beats[i].at).toBeGreaterThan(beats[i - 1].at);
  });
});

describe('seeding guides', () => {
  const load = id => seedingReplay.parseSeedingTrace(read(`data/seeding/${id}.json`));
  it('walks the nominal sortie from climb through landing', () => {
    const trace = load('nominal');
    const beats = G.seedingGuideBeats(trace);
    expect(beats.map(b => b.title)).toEqual([
      'Taking off', 'Climbing to the release point', 'Holding for the release decision', 'The valve is open',
      'Release finished', 'Handing off to the marker', 'Landed',
    ]);
    expect(beats.find(b => b.title === 'The valve is open').text).toContain(trace.release.amount_kg.toFixed(3));
    expect(beats.at(-1).text).toContain(trace.touchdown_horizontal_error_m.toFixed(3));
    expect(beats.at(-1).at).toBeCloseTo(trace.states.at(-1).t - trace.frames[0].t, 6);
  });

  it('states the recorded denial and the recovery', () => {
    for (const id of ['weather_denial', 'airspace_denial', 'traffic_conflict', 'clearance_expired', 'altitude_denial']) {
      const trace = load(id);
      const beats = G.seedingGuideBeats(trace);
      const denied = trace.states.find(s => s.to === 'return_to_recovery');
      const beat = beats.find(b => b.title === 'Release denied');
      expect(beat, id).toBeTruthy();
      expect(beat.text).toContain(denied.reason);
      expect(beats.some(b => b.title === 'The valve is open')).toBe(false);
      expect(beats.at(-1).title).toBe('Recovered');
    }
  });

  it('says why the marker-loss landing stopped', () => {
    const trace = load('marker_loss');
    const beats = G.seedingGuideBeats(trace);
    const failed = beats.find(b => b.title === 'Recovery failed');
    expect(failed.text).toContain('marker_lost_during_descent');
    expect(beats.some(b => b.title === 'The valve is open')).toBe(true);
  });
});

describe('slam guide', () => {
  const trace = slamReplay.parseSlamTrace(read('data/slam_trace.json'));
  const beats = G.slamGuideBeats(trace);
  it('marks the injected drift and the recorded end error', () => {
    expect(beats[0].text).toContain(`${trace.report.drift_injected.yaw_deg_per_s}`);
    const samples = trace.samples.drifting_odometry;
    const end = samples[samples.length - 1];
    const odom = Math.hypot(end[4] - end[1], end[5] - end[2]);
    expect(beats.at(-1).text).toContain(odom.toFixed(1));
    expect(beats.at(-1).at).toBeCloseTo(end[0] - samples[0][0], 6);
    for (let i = 1; i < beats.length; i++) expect(beats[i].at).toBeGreaterThan(beats[i - 1].at);
  });
});
