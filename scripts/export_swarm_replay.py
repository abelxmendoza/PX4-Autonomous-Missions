#!/usr/bin/env python3
"""Package synchronized, timestamped Gazebo swarm evidence for the browser."""
import argparse
import json
from pathlib import Path


def export(path):
    records = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    header = records[0]
    samples = [r for r in records if r['type'] == 'sample' and len(r['vehicles']) == 2]
    if len(samples) < 2:
        raise ValueError('Recording must contain both vehicles')
    frames = []
    index = 0
    start = samples[0]['time']
    for tick in range(int((samples[-1]['time'] - start) * 10) + 1):
        stamp = start + tick / 10
        while index + 1 < len(samples) and samples[index + 1]['time'] <= stamp:
            index += 1
        s = samples[index]
        vehicles = {}
        for name, v in s['vehicles'].items():
            vehicles[name] = {'position': v['position'], 'state': v['state'],
                              'valid': v['valid'] and 0 <= stamp - v['sent'] <= .75}
        frames.append({'time': tick / 10, 'phase': s['phase'], 'reason': s['reason'],
                       'completed': len(s['completed']), 'vehicles': vehicles})
    proof = path.parent / 'verification.json'
    return {'schema': 1, 'source': path.parent.name, 'homes': header['homes_ned'],
            'taskCount': len(header['expected_tasks']),
            'verification': json.loads(proof.read_text()) if proof.exists() else None,
            'frames': frames}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--output', type=Path, default=Path('web/replay/data/swarm.json'))
    args = parser.parse_args()
    args.output.write_text(json.dumps(export(args.log), separators=(',', ':'), allow_nan=False) + '\n')
