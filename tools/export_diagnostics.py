"""Export both 400 Hz IMUs and 100 Hz controller diagnostics with bounded memory.
Usage: python tools/export_diagnostics.py FLT00000.BIN --out flight
"""
import argparse
import csv
import json
import struct
from contextlib import ExitStack
from pathlib import Path
from log_container import records, IMU_FMT, CONTROL_FMT

def export(path, prefix):
    stats = {}
    imu_fields = ['host_us', 'sequence', 'sensor_clock', 'calibrated']
    axes = ['gx_dps', 'gy_dps', 'gz_dps', 'ax_g', 'ay_g', 'az_g']
    imu_fields += ['raw_'+a for a in axes]+['filtered_'+a for a in axes]
    ctrl_fields = ['host_us', 'demand_p_dps', 'demand_q_dps', 'p_dps', 'q_dps', 'r_dps',
                   'aileron', 'elevator', 'rudder', 'throttle', 'mode', 'armed', 'failsafe', 'integrators']
    with ExitStack() as stack:
        writers = []
        for suffix, fields in [('imu0', imu_fields), ('imu1', imu_fields), ('control', ctrl_fields)]:
            f = stack.enter_context(open(f'{prefix}_{suffix}.csv', 'w', newline=''))
            w = csv.writer(f); w.writerow(fields); writers.append(w)
        for magic, raw in records(path, stats):
            if magic == 0xA16D:
                v = struct.unpack(IMU_FMT, raw)
                sensor, flags = v[-3:-1]
                if sensor > 1:
                    continue
                values = [v[4+i]/(16 if i % 6 < 3 else 2048) for i in range(12)]
                writers[sensor].writerow([v[1], v[2], v[3], flags & 1]+values)
                stats[f'imu{sensor}_records'] = stats.get(f'imu{sensor}_records', 0)+1
            elif magic == 0xC17D:
                v = struct.unpack(CONTROL_FMT, raw)
                flags = v[-2]
                values = [x/16 for x in v[2:7]]+[x/10000 for x in v[7:11]]
                writers[2].writerow([v[1]]+values+[v[11], flags & 1, (flags >> 1) & 1, (flags >> 2) & 1])
                stats['control_records'] = stats.get('control_records', 0)+1
    Path(f'{prefix}_recovery.json').write_text(json.dumps(stats, indent=2))
    return stats

if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('file'); ap.add_argument('--out', required=True)
    args = ap.parse_args()
    print(json.dumps(export(args.file, args.out), indent=2))
