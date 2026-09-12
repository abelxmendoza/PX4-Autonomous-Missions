import {describe, it, expect} from 'vitest';
import {createRequire} from 'node:module';
const require = createRequire(import.meta.url);
const {parseSwarmReplay} = require('../swarm-replay.js');
const recording = require('../data/swarm.json');
describe('synchronized swarm recording', () => {
  it('contains independent moving tracks and verified completion', () => {
    const data = parseSwarmReplay(JSON.stringify(recording));
    expect(data.verification.passed).toBe(true);
    expect(data.frames.at(-1).phase).toBe('COMPLETE');
    for (const id of ['px4_1', 'px4_2']) {
      const positions = data.frames.filter(f => f.vehicles[id].valid).map(f => f.vehicles[id].position);
      expect(Math.max(...positions.map(p => p[0])) - Math.min(...positions.map(p => p[0]))).toBeGreaterThan(5);
    }
    expect(data.frames[1].time - data.frames[0].time).toBeCloseTo(.1);
  });
  it('rejects incomplete or non-monotonic timelines', () => {
    const bad = structuredClone(recording);
    bad.frames[1].time = bad.frames[0].time;
    expect(() => parseSwarmReplay(JSON.stringify(bad))).toThrow();
    delete bad.frames[0].vehicles.px4_2;
    expect(() => parseSwarmReplay(JSON.stringify(bad))).toThrow();
  });
});
