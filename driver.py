#!/usr/bin/env python3
import serial, time, statistics

PORT = "/dev/ttyUSB0"
BAUD = 115200
SAMPLES_PER_PHASE = 60     # ~1 second if your stream is ~60 Hz
DT = 0.005                 # small delay between reads

def to_s16(v: int) -> int:
    v = int(v)
    if v > 32767: v -= 65536
    if v < -32768: v += 65536
    return v

def read_frame(ser):
    """
    Parse a line like '0:gx,gy,gz|1:gx,gy,gz|2:gx,gy,gz'
    Return [[gx,gy,gz], [gx,gy,gz], [gx,gy,gz]] as signed ints.
    """
    line = ser.readline().decode("utf-8", errors="ignore").strip()
    if not line or ":" not in line: 
        return None
    try:
        gyros = []
        for part in line.split("|"):
            if ":" not in part: 
                continue
            _, vec = part.split(":", 1)
            gx, gy, gz = (to_s16(x) for x in map(int, vec.split(",")))
            gyros.append([gx, gy, gz])
        if len(gyros) >= 3:
            return gyros[:3]
    except Exception:
        pass
    return None

def grab_phase(ser, n=SAMPLES_PER_PHASE, dt=DT):
    vals = []  # list of [[gx,gy,gz],[...],[...]]
    while len(vals) < n:
        fr = read_frame(ser)
        if fr:
            vals.append(fr)
        time.sleep(dt)
    return vals

def medians_per_axis(frames):
    """Return 3x3 medians: med[gyro][axis]."""
    med = []
    for g in range(3):
        axes = []
        for a in range(3):
            axes.append(statistics.median([f[g][a] for f in frames]))
        med.append(axes)
    return med

def fmt(x): 
    return f"{x:7.1f}"

def analyze_pedal(name, ser):
    print(f"\n--- {name.upper()} ---")
    print(f"[{name}] PRESS fully and hold...")
    press = grab_phase(ser)
    print(f"[{name}] RELEASE fully...")
    rel = grab_phase(ser)

    press_med = medians_per_axis(press)   # 3x3
    rel_med   = medians_per_axis(rel)     # 3x3

    # delta = press - release (signed), mag = |delta|
    delta = [[press_med[g][a] - rel_med[g][a] for a in range(3)] for g in range(3)]
    mag   = [[abs(delta[g][a]) for a in range(3)] for g in range(3)]

    # find best (g,a) by magnitude
    best_g, best_a, best_mag = 0, 0, -1
    for g in range(3):
        for a in range(3):
            if mag[g][a] > best_mag:
                best_g, best_a, best_mag = g, a, mag[g][a]

    # polarity & min/max suggestion:
    # if press > release → normal (min=release, max=press)
    # else inverted (min=release, max=press still works with normalize handling)
    press_val = press_med[best_g][best_a]
    rel_val   = rel_med[best_g][best_a]
    suggested_min, suggested_max = rel_val, press_val  # driver can handle inverted too

    # print table
    print("\nPer-gyro per-axis medians (press / release) and delta:")
    axes_lbl = ["X", "Y", "Z"]
    for g in range(3):
        row_p = "  G{} press : ".format(g) + " ".join(f"{axes_lbl[a]}={fmt(press_med[g][a])}" for a in range(3))
        row_r = "  G{} release: ".format(g) + " ".join(f"{axes_lbl[a]}={fmt(rel_med[g][a])}"   for a in range(3))
        row_d = "  G{} delta  : ".format(g) + " ".join(f"{axes_lbl[a]}={fmt(delta[g][a])}"     for a in range(3))
        print(row_p); print(row_r); print(row_d)

    print(f"\n[{name}] BEST → gyro={best_g}, axis={best_a} ({axes_lbl[best_a]}), |delta|={best_mag:.1f}")
    print(f"[{name}] Suggested range: min={suggested_min:.1f}, max={suggested_max:.1f} (press-release medians)")

    return {
        "name": name,
        "gyro": best_g,
        "axis": best_a,
        "min":  suggested_min,
        "max":  suggested_max,
        "delta": best_mag,
    }

def main():
    print(f"Opening {PORT} @ {BAUD} …")
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        time.sleep(1)
        print("Reading a few frames to stabilize…")
        _ = grab_phase(ser, n=20, dt=0.005)

        results = []
        for pedal in ("clutch", "brake", "gas"):
            results.append(analyze_pedal(pedal, ser))

        print("\n===== SUMMARY =====")
        for r in results:
            print(f"{r['name']:6s} → gyro={r['gyro']}, axis={r['axis']}, "
                  f"min={r['min']:.1f}, max={r['max']:.1f}, |delta|={r['delta']:.1f}")

        # Recommended PEDAL_MAP and ranges block to paste into your driver:
        print("\nRecommended PEDAL_MAP (paste into driver):")
        print("PEDAL_MAP = {")
        for r in results:
            print(f"    '{r['name']}': ({r['gyro']}, {r['axis']}),")
        print("}")

        print("\nRecommended ranges (for optional preloading instead of recalib):")
        for r in results:
            print(f"{r['name']}: min={r['min']:.1f}, max={r['max']:.1f}")

        print("\nNote: If a pedal feels inverted in the driver, just swap min/max there "
              "(the normalize function supports inverted ranges).")

if __name__ == '__main__':
    main()
