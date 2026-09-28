"""Measured vibration spectrum from exported IMU CSV (requires numpy).
python tools/analyze_imu.py flight_imu0.csv --out spectrum.csv
Does not change firmware or recommend a notch from a single peak automatically.
"""
import argparse
import csv
import numpy as np

def spectrum(path, out):
    data = np.genfromtxt(path, delimiter=',', names=True)
    if len(data) < 1024:
        raise ValueError('At least 1024 samples required')
    seq = data['sequence'].astype(np.int64)
    if np.any(np.diff(seq) != 1):
        raise ValueError('Dropped samples: select a contiguous sequence before FFT')
    # FIFO production clock is 400 Hz. Retrieval timestamps are approximate.
    fs, n = 400., 1024
    win = np.hanning(n)
    names = [f'{kind}_g{axis}_dps' for kind in ('raw', 'filtered') for axis in 'xyz']
    estimates = []
    for name in names:
        powers = []
        for start in range(0, len(data)-n+1, n//2):
            x = data[name][start:start+n]
            fft = np.fft.rfft((x-x.mean())*win)
            p = np.abs(fft)**2/(fs*np.sum(win**2)); p[1:-1] *= 2
            powers.append(p)
        estimates.append(np.mean(powers, axis=0))
    with open(out, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['frequency_hz']+[n+'_psd' for n in names])
        w.writerows(zip(np.fft.rfftfreq(n, 1/fs), *estimates))
    print('Spectrum exported; Nyquist is 200 Hz. Sensor filtering precedes these raw values.')

if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('file'); ap.add_argument('--out', required=True)
    args = ap.parse_args(); spectrum(args.file, args.out)
