// Cloud Seeding replay logic, checked against the committed recorded sorties.
import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { createRequire } from 'node:module';

const require = createRequire(import.meta.url);
const R = require('../seeding-replay.js');
const SITE = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const index = JSON.parse(readFileSync(join(SITE, 'data/seeding/index.json'), 'utf8'));
const truth = JSON.parse(readFileSync(join(SITE, 'data/search_targets.json'), 'utf8'));
const load = id => R.parseSeedingTrace(readFileSync(join(SITE, 'data/seeding', `${id}.json`), 'utf8'));
const DENIALS = {weather_denial: 'weather', airspace_denial: 'airspace', traffic_conflict: 'traffic',
                 clearance_expired: 'comms', altitude_denial: 'altitude'};

describe('seeding replay data', () => {
  it('offers all seven scenarios, each a byte-identical copy of the committed evidence', () => {
    expect(index.scenarios.map(s => s.id).sort()).toEqual(
      ['airspace_denial', 'altitude_denial', 'clearance_expired', 'marker_loss', 'nominal', 'traffic_conflict', 'weather_denial']);
    for (const s of index.scenarios) {
      const site = readFileSync(join(SITE, 'data/seeding', s.file), 'utf8');
      const evidence = readFileSync(join(SITE, '../../', index.source, s.file), 'utf8');
      expect(site, s.id).toBe(evidence);
    }
  });

  it('every recorded sortie ended the way its scenario is designed to', () => {
    for (const s of index.scenarios) expect(s.result_as_designed, s.id).toBe(true);
  });
});

describe('seeding replay logic', () => {
  it('the nominal sortie releases seed with every gate open and lands on the pad', () => {
    const tr = load('nominal');
    const decided = R.releaseDecisionTime(tr);
    expect(R.gatesAt(tr, decided - 0.01).every(g => g.status === 'pending')).toBe(true);
    expect(R.gatesAt(tr, decided).every(g => g.status === 'pass')).toBe(true);
    const end = tr.frames[tr.frames.length - 1].t;
    expect(R.releasedAt(tr, end)).toBeCloseTo(tr.release.amount_kg, 6);
    expect(R.releasedAt(tr, 0)).toBe(0);
    expect(tr.frames.some(f => f.dispense)).toBe(true);
    expect(R.stateAt(tr, end).state).toBe('landed');
    const score = R.touchdownScore(tr, truth);
    expect(score.errorM).toBeLessThan(0.5);
    expect(score.errorM).toBeCloseTo(tr.touchdown_horizontal_error_m, 2);
  });

  for (const [id, gate] of Object.entries(DENIALS)) {
    it(`${id}: only the ${gate} gate denies, nothing is dispensed, and the drone still lands`, () => {
      const tr = load(id);
      const gates = R.gatesAt(tr, Infinity);
      expect(gates.filter(g => g.status === 'fail').map(g => g.id)).toEqual([gate]);
      expect(gates.find(g => g.id === gate).reason).not.toBe('');
      expect(tr.frames.some(f => f.dispense)).toBe(false);
      expect(R.releasedAt(tr, Infinity)).toBe(0);
      expect(R.stateAt(tr, Infinity).state).toBe('aborted');
      expect(R.touchdownScore(tr, truth).errorM).toBeLessThan(0.5);
    });
  }

  it('marker loss fails safe during the landing and says why', () => {
    const tr = load('marker_loss');
    expect(R.stateAt(tr, Infinity).state).toBe('failed');
    expect(R.outcomeText(tr)).toMatch(/recovery failed: marker_lost_during_descent/);
  });

  it('phases run in order along the recorded path and the pose interpolates between samples', () => {
    const tr = load('nominal');
    const order = tr.frames.map(f => R.phaseIndex(f.state));
    for (let i = 1; i < order.length; i++) expect(order[i]).toBeGreaterThanOrEqual(order[i - 1]);
    const [a, b] = tr.frames.slice(100, 102);
    const mid = R.seedingPoseAt(tr.frames, (a.t + b.t) / 2);
    expect(mid.n).toBeCloseTo((a.n + b.n) / 2, 6);
  });

  it('rejects a document that is not a seeding trace', () => {
    expect(() => R.parseSeedingTrace('{"schema":1,"kind":"search"}')).toThrow();
  });
});
