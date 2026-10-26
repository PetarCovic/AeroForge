"""Generate explicitly SYNTHETIC AeroForge GUI test telemetry.

This file is not output from the AeroForge physics engine.
Usage: python tools/make_visualization_demo_csv.py
"""
import csv
import math
from pathlib import Path

OUTPUT = Path(__file__).resolve().parents[1] / 'examples' / 'data'
COLUMNS = [
    'simulationTimeS',
    *[f'truthPositionN.{k}' for k in 'xyz'],
    *[f'truthVelocityN.{k}' for k in 'xyz'],
    *[f'truthAttitudeNB.{k}' for k in 'wxyz'],
    *[f'truthAngularVelocityB.{k}' for k in 'xyz'],
    *[f'estimatedPositionN.{k}' for k in 'xyz'],
    *[f'estimatedVelocityN.{k}' for k in 'xyz'],
    *[f'estimatedAttitudeNB.{k}' for k in 'wxyz'],
    *[f'estimatedAngularVelocityB.{k}' for k in 'xyz'],
    *[f'attitudeErrorB.{k}' for k in 'xyz'],
    *[f'commandedMomentB.{k}' for k in 'xyz'],
    *[f'actualMomentB.{k}' for k in 'xyz'],
]


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    path = OUTPUT / 'visualization_demo_SYNTHETIC.csv'
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(COLUMNS)
        for i in range(601):
            t = i * .05
            east = 2 * math.sin(.5 * t)
            yaw = .2 * math.sin(.4 * t)
            q = (math.cos(yaw / 2), 0, 0, math.sin(yaw / 2))
            row = [
                t, 5 * t, east, -.3 * t,
                5, math.cos(.5 * t), -.3,
                *q, 0, 0, .08 * math.cos(.4 * t),
                5 * t + .1, east - .1, -.3 * t,
                5, math.cos(.5 * t), -.3,
                *q, 0, 0, .08 * math.cos(.4 * t),
                0, 0, 0,
                0, 0, 0, 0, 0, 0,
            ]
            assert len(row) == len(COLUMNS)
            writer.writerow(row)
    events = OUTPUT / 'events_demo_SYNTHETIC.csv'
    with events.open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['simulationTimeS', 'type', 'message'])
        writer.writerow([0, 'INFO', 'Synthetic test recording started'])
        writer.writerow([15, 'INFO', 'Synthetic test midpoint'])
        writer.writerow([30, 'INFO', 'Synthetic test recording ended'])
    print(f'Generated: {path}')
    print(f'Generated: {events}')


if __name__ == '__main__':
    main()
