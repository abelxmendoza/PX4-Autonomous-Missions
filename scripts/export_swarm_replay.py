#!/usr/bin/env python3
"""Package synchronized, timestamped Gazebo swarm evidence for the browser."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/px4_offboard'))
from px4_offboard.swarm_verify import verify


def export(path):
    records = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    header = records[0]
    samples = [r for r in records if r['type'] == 'sample' and len(r['vehicles']) == 2]
    if len(samples) < 2:
        raise ValueError('Recording must contain both vehicles')
    frames = []
    index = 0
    start = samples[0]['time']
    duration = samples[-1]['time'] - start
    offsets = [tick / 10 for tick in range(int(duration * 10) + 1)]
    # The exact terminal sample usually lies between 10 Hz playback frames.
    # Preserve it rather than rounding down and losing COMPLETE/landed flags.
    if offsets[-1] < duration:
        offsets.append(duration)
    for offset in offsets:
        stamp = start + offset
        while index + 1 < len(samples) and samples[index + 1]['time'] <= stamp:
            index += 1
        s = samples[index]
        vehicles = {}
        for name, v in s['vehicles'].items():
            vehicles[name] = {'position': v['position'], 'state': v['state'],
                              'valid': v['valid'] and 0 <= stamp - v['sent'] <= .75}
        frames.append({'time': offset, 'phase': s['phase'], 'reason': s['reason'],
                       'completed': len(s['completed']), 'vehicles': vehicles})
    return {'schema': 1, 'source': path.parent.name, 'homes': header['homes_ned'],
            'taskCount': len(header['expected_tasks']),
            'verification': verify(path),
            'frames': frames}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--output', type=Path, default=Path('web/replay/data/swarm.json'))
    args = parser.parse_args()
    args.output.write_text(json.dumps(export(args.log), separators=(',', ':'), allow_nan=False) + '\n')
