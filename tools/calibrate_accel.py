"""Fit per-axis six-face accelerometer offset/scale from stationary CSV means.
Input CSV: face,ax_g,ay_g,az_g; faces +X,-X,+Y,-Y,+Z,-Z in the AHRS body frame.
Values must be BEFORE applying stored accelerometer calibration.
Outputs values for config::accel_offset_g / accel_scale; never writes firmware.
"""
import argparse
import csv
import json
import math

def fit(path):
    groups = {f'{s}{a}': [] for a in 'XYZ' for s in '+-'}
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            face = row['face'].upper()
            values = [float(row[k]) for k in ('ax_g', 'ay_g', 'az_g')]
            if face not in groups or not all(math.isfinite(v) for v in values):
                raise ValueError('Invalid face or measurement')
            groups[face].append(values)
    offset, scale = [], []
    for i, axis in enumerate('XYZ'):
        means = []
        for sign in '+-':
            rows = groups[sign+axis]
            if len(rows) < 100:
                raise ValueError(f'Need at least 100 stationary samples for {sign+axis}')
            for j in range(3):
                vals = [r[j] for r in rows]
                mean = sum(vals)/len(vals)
                rms = math.sqrt(sum((v-mean)**2 for v in vals)/len(vals))
                if rms > .02 or (j != i and abs(mean) > .15):
                    raise ValueError(f'Movement or incorrect face alignment: {sign+axis}')
            means.append(sum(r[i] for r in rows)/len(rows))
        plus, minus = means
        span = plus-minus
        if not 1.6 < span < 2.4:
            raise ValueError(f'Unexpected +/-{axis} gravity span')
        offset.append((plus+minus)/2); scale.append(2/span)
    return {'offset_g': offset, 'scale': scale}

if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('file'); args = ap.parse_args()
    print(json.dumps(fit(args.file), indent=2))
