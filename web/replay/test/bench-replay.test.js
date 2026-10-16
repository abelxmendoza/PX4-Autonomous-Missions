import { describe, it, expect } from 'vitest';
import { createRequire } from 'node:module';
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
