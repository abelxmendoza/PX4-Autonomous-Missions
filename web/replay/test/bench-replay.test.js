import { describe, it, expect } from 'vitest';
import { createRequire } from 'node:module';
import { readFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
const SITE = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const require = createRequire(import.meta.url);
const { parseBenchCampaign, sampleAt, faultWindowsFor } = require('../bench-replay.js');

function sample(t, overrides = {}) {
  return {
    t, command_pct: 50, rpm: 4000, thrust_n: 5, load_cell_n: 5,
    voltage_v: 16, temp_c: 30, fault_active: '', ...overrides,
  };
}

describe('parseBenchCampaign', () => {
  it('parses a well-formed payload', () => {
    const data = parseBenchCampaign(JSON.stringify({ report: { passed: true }, samples: [sample(0)] }));
    expect(data.report.passed).toBe(true);
    expect(data.samples).toHaveLength(1);
  });

  it('rejects a payload missing report or samples', () => {
    expect(() => parseBenchCampaign(JSON.stringify({ samples: [sample(0)] }))).toThrow();
    expect(() => parseBenchCampaign(JSON.stringify({ report: {} }))).toThrow();
  });

  it('rejects an empty sample list', () => {
    expect(() => parseBenchCampaign(JSON.stringify({ report: {}, samples: [] }))).toThrow();
  });
});

describe('sampleAt', () => {
  const samples = [
    sample(0, { thrust_n: 0, command_pct: 0 }),
    sample(1, { thrust_n: 10, command_pct: 100 }),
    sample(2, { thrust_n: 20, command_pct: 100 }),
  ];

  it('interpolates numeric fields between the bracketing samples', () => {
    const mid = sampleAt(samples, 0.5);
    expect(mid.thrust_n).toBeCloseTo(5, 5);
    expect(mid.command_pct).toBeCloseTo(50, 5);
  });

  it('clamps before the first and after the last sample', () => {
    expect(sampleAt(samples, -5).thrust_n).toBe(0);
    expect(sampleAt(samples, 50).thrust_n).toBe(20);
  });

  it('takes fault_active from the earlier bracketing sample, not interpolated', () => {
    const faulty = [sample(0, { fault_active: 'load_cell_dropout' }), sample(1, { fault_active: '' })];
    expect(sampleAt(faulty, 0.9).fault_active).toBe('load_cell_dropout');
  });
});

describe('faultWindowsFor', () => {
  it('groups consecutive same-name faulty samples into one window', () => {
    const samples = [
      sample(0, { fault_active: '' }),
      sample(1, { fault_active: 'load_cell_dropout' }),
      sample(2, { fault_active: 'load_cell_dropout' }),
      sample(3, { fault_active: '' }),
      sample(4, { fault_active: 'frozen_temp_sensor' }),
    ];
    const windows = faultWindowsFor(samples);
    expect(windows).toEqual([
      { name: 'load_cell_dropout', start_s: 1, end_s: 2 },
      { name: 'frozen_temp_sensor', start_s: 4, end_s: 4 },
    ]);
  });

  it('returns no windows when nothing ever faults', () => {
    expect(faultWindowsFor([sample(0), sample(1)])).toEqual([]);
  });
});

describe('bench narration', () => {
  const { benchStory, throttleSteps, BENCH_PLAIN } = require('../bench-replay.js');
  const data = parseBenchCampaign(readFileSync(join(SITE, 'data/bench_campaign.json'), 'utf8'));
  const end = data.samples[data.samples.length - 1].t;

  it('tells the story from the recording: ready, throttle steps, faults, the catch, and the result', () => {
    expect(benchStory(data, 0).title).toBe('Ready');
    expect(throttleSteps(data.samples).map(s => s.to)).toEqual([30, 60, 90, 50, 0]);
    expect(benchStory(data, 3).title).toBe('Throttle up to 30%');
    expect(benchStory(data, 8).title).toMatch(/Sensor failure/);
    expect(benchStory(data, 11.2).title).toMatch(/Temperature sensor frozen/);
    expect(benchStory(data, 12).title).toBe('Frozen sensor caught at 11.5 s');
    expect(benchStory(data, 10.5).title).toMatch(/Full power/);
    expect(benchStory(data, 18).title).toBe('Motor off');
    const done = benchStory(data, end);
    expect(done.title).toBe('Test complete');
    expect(done.text).toMatch(/5 of 5 automatic checks passed/);
  });

  it('has a plain-English name for every check in the report', () => {
    for (const r of data.report.requirements) expect(BENCH_PLAIN[r.id], r.id).toBeTruthy();
  });
});
