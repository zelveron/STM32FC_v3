#!/usr/bin/env python3
"""Fit BMM350 hard/soft iron and axis alignment from Record USB MAG + ATT data.

Requires NumPy. Does not open USB, alter firmware or install calibration.
Use a gyro-only yaw capture with slow, full 3D rotations and motors off.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import numpy as np


def read_capture(path):
    fields, attitudes = [], []
    attitude = None
    eligible = False
    bias_ready = False
    gyro_only = True  # Older firmware has gyro-only yaw and no YAW_STATUS.
    att_count = 0
    health_at = bias_at = -1000
    yaw_at = None
    mag_since_att = 0
    for line in Path(path).read_text(encoding="utf-8", errors="replace").splitlines():
        p = line.strip().split(",")
        try:
            if p[0] == "YAW_STATUS":
                kv = dict(x.split("=", 1) for x in p[1:] if "=" in x)
                gyro_only = kv.get("source") == "GYRO"
                yaw_at = att_count
            elif p[0] == "IMU_HEALTH" and len(p) == 6:
                eligible = p[4] == "0" and p[5] == "1"
                health_at = att_count
                if not eligible: attitude = None
            elif p[0] == "EST":
                kv = dict(x.split("=", 1) for x in p[1:] if "=" in x)
                bias_ready = kv.get("bias_ready") == "1"
                bias_at = att_count
            elif p[0] == "ATT" and len(p) == 4:
                att_count += 1
                a = np.array([float(x) for x in p[1:]])
                attitude = a if np.isfinite(a).all() else None
                mag_since_att = 0
            elif p[0] == "MAG" and len(p) == 4:
                mag_since_att += 1
                m = np.array([float(x) for x in p[1:]])
                # Capture CSV has no timestamps. Bound health age by the 20 Hz
                # attitude sequence and never reuse one ATT indefinitely.
                fresh = att_count-health_at <= 60 and att_count-bias_at <= 60 and (yaw_at is None or att_count-yaw_at <= 60)
                if eligible and bias_ready and fresh and gyro_only and attitude is not None and mag_since_att <= 2 and np.isfinite(m).all():
                    fields.append(m); attitudes.append(attitude.copy())
        except ValueError:
            continue
    return np.asarray(fields), np.asarray(attitudes)


def world_rotations(attitudes):
    r, p, y = np.radians(attitudes).T
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.stack((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr,
                     sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr,
                     -sp, cp*sr, cp*cr), axis=1).reshape(-1, 3, 3)


def proper_rotations():
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((-1, 1), repeat=3):
            matrix = np.zeros((3, 3))
            for i, (j, sign) in enumerate(zip(perm, signs)): matrix[i, j] = sign
            if np.linalg.det(matrix) > .5:
                yield [sign*(j+1) for j, sign in zip(perm, signs)], matrix


def fit(fields, attitudes, field_ut=50.0):
    fields, attitudes = np.asarray(fields, dtype=float), np.asarray(attitudes, dtype=float)
    if fields.ndim != 2 or fields.shape[1:] != (3,) or len(fields) < 300 or attitudes.shape != fields.shape:
        raise ValueError("Need at least 300 paired MAG/ATT samples from full 3D rotations")
    if not np.isfinite(fields).all() or not np.isfinite(attitudes).all() or not 15 <= field_ut <= 100:
        raise ValueError("Invalid samples or normalization field")
    # Normalize before solving x'Ax + g'x = 1 for numerical conditioning.
    origin = fields.mean(axis=0)
    scale = np.sqrt(np.mean((fields-origin)**2))
    if scale < 1: raise ValueError("Insufficient magnetic motion")
    x, y, z = ((fields-origin)/scale).T
    design = np.column_stack((x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z, x, y, z))
    coeff, _, rank, singular = np.linalg.lstsq(design, np.ones(len(x)), rcond=None)
    if rank != 9 or singular[0]/singular[-1] > 1000:
        raise ValueError("Degenerate capture; rotate through all three axes")
    a, b, c, d, e, f, g, h, i = coeff
    matrix = np.array(((a,d,e),(d,b,f),(e,f,c)))
    center = -.5*np.linalg.solve(matrix, (g,h,i))
    shape = matrix/(1+center @ matrix @ center)
    eigenvalues, eigenvectors = np.linalg.eigh(shape)
    if eigenvalues.min() <= 0 or eigenvalues.max()/eigenvalues.min() > 25:
        raise ValueError("Calibration is not a plausible positive ellipsoid")
    offset = origin + scale*center
    correction = field_ut/scale * (eigenvectors @ np.diag(np.sqrt(eigenvalues)) @ eigenvectors.T)
    if (np.max(np.abs(offset)) > 2000 or np.max(np.abs(correction)) > 10 or
            correction[0,0] <= .01 or np.linalg.det(correction[:2,:2]) <= .0001 or np.linalg.det(correction) <= .001):
        raise ValueError("Calibration exceeds firmware limits")
    corrected = (fields-offset) @ correction.T
    norm = np.linalg.norm(corrected, axis=1)
    radial_error = np.abs(norm/field_ut-1)
    if np.sqrt(np.mean(radial_error**2)) > .05 or np.quantile(radial_error,.99) > .12:
        raise ValueError("Field is inconsistent; remove interference and record again")
    directions = corrected/norm[:,None]
    if np.linalg.eigvalsh(directions.T @ directions/len(fields)).min() < .15:
        raise ValueError("Poor 3D coverage; planar rotation is insufficient")
    if any(np.mean(directions[:,j] > .7) < .025 or np.mean(directions[:,j] < -.7) < .025 for j in range(3)):
        raise ValueError("Missing opposite orientations in the rotation capture")
    rotations = world_rotations(attitudes)
    candidates = []
    for axes, sensor_to_body in proper_rotations():
        world = np.einsum('nij,nj->ni', rotations, corrected @ sensor_to_body.T)
        mean = world.mean(axis=0)
        # The correct mapping leaves the Earth field fixed despite body motion.
        rms = np.sqrt(np.mean(np.sum((world-mean)**2, axis=1)))/field_ut
        candidates.append((float(rms), axes))
    candidates.sort()
    best, second = candidates[:2]
    if best[0] > .12 or second[0] < max(.20, best[0]*2):
        raise ValueError("Axis mapping is ambiguous or disagrees with attitude; rotate slowly through all axes with gyro-only yaw")
    return {"rotation": best[1], "offset_ut": offset.tolist(), "correction": correction.tolist(),
            "field_ut": float(field_ut), "samples": len(fields),
            "radial_rms": float(np.sqrt(np.mean(radial_error**2))),
            "alignment_rms": best[0], "second_alignment_rms": second[0]}


def header(result):
    def number(v):
        text = f"{v:.8g}"
        return (text if "." in text or "e" in text else text+".0")+"f"
    rows = ",".join("{"+",".join(number(v) for v in row)+"}" for row in result["correction"])
    return ("#pragma once\n#include \"../estimation/mag_heading.hpp\"\nnamespace config {\n"
            "// Measured by tools/calibrate_mag.py; verify heading and motor interference.\n"
            "inline constexpr estimation::MagHeadingConfig magnetic_calibration{\n"
            "    true, true, true,\n    {"+",".join(map(str,result["rotation"]))+"},\n    {"+
            ",".join(number(v) for v in result["offset_ut"])+"},\n    {"+rows+"},\n    "+
            number(result["field_ut"])+"\n};\n}\n")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path, required=True, help="new directory for report and configuration header")
    parser.add_argument("--field-ut", type=float, default=50, help="normalization radius, not measured local field strength")
    args=parser.parse_args()
    try:
        fields, attitudes=read_capture(args.capture)
        result=fit(fields, attitudes, args.field_ut)
        result["capture_sha256"]=hashlib.sha256(args.capture.read_bytes()).hexdigest()
        args.output.mkdir(parents=True, exist_ok=False)
        (args.output/"magnetometer.hpp").write_text(header(result), encoding="utf-8")
        (args.output/"report.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
    except (ValueError,OSError,np.linalg.LinAlgError) as error:
        parser.exit(1,f"Calibration rejected: {error}\n")
    print(f"Calibration and axis mapping saved to {args.output}. Review report.json, then install magnetometer.hpp and rebuild firmware.")


if __name__ == "__main__": main()
