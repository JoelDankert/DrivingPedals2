#!/usr/bin/env python3

import argparse
import ast
import glob
import math
import os
import subprocess
import sys
import time
from typing import List

# ---- Optional imports with friendly messages ----
try:
    import serial
except Exception:
    print("Error: missing pyserial (python3-serial). Install with: sudo apt install python3-serial")
    sys.exit(1)

try:
    from evdev import UInput, AbsInfo, ecodes
except Exception:
    print("Error: missing python-evdev. Install with: sudo apt install python3-evdev")
    sys.exit(1)

# --------- Config ---------
DEFAULT_BAUD = 115200
ABS_MIN = 0
ABS_MAX = 255  # exported axis resolution (0..255)
AXES = [ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_RZ]
LABELS = ["Clutch", "Gas", "Brake"]
BAR_WIDTH = 40
ALPHA = 0.2  # EMA base alpha for smoothing (0..1) — lower = smoother  # EMA base alpha for smoothing (0..1) — lower = smoother
# Top deadzone: cut off the top N percent (e.g. 3 means values >= 97% snap to 100%)
DEADZONE_TOP_PERCENT = 10.0
# Bottom deadzone (keep small; set to 0 to disable). We're only cutting the top by default.
DEADZONE_BOTTOM_PERCENT = 10.0
SERIAL_TIMEOUT = 1.0
REDRAW_INTERVAL = 0.02  # ~50 Hz UI update
# How aggressively the adaptive alpha should scale with change magnitude.
# Larger values make alpha reach 1.0 sooner as the difference increases.
ADAPTIVE_SENSITIVITY = 0.5
# Minimum percent diff before adaptive alpha starts to increase
DIFF_THRESHOLD = 10.0

# --------- Helpers ---------

def modprobe_uinput():
    """Try to load uinput kernel module if not present."""
    if not os.path.exists('/dev/uinput'):
        try:
            subprocess.run(['modprobe', 'uinput'], check=False)
            time.sleep(0.1)
        except Exception:
            pass


def find_serial_port() -> str:
    """Auto-detect a serial port if present (ttyUSB*, ttyACM*). Returns a path or default.

    Priority: /dev/ttyUSB*, then /dev/ttyACM*, else fall back to /dev/ttyUSB0.
    """
    candidates = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    candidates = [c for c in candidates if os.path.exists(c)]
    return candidates[0] if candidates else '/dev/ttyUSB0'


def clear_screen():
    # print real ANSI escape sequences so terminals render colors correctly
    # use  (octal) which is unambiguous in many environments
    print('\x1b[2J\x1b[H', end='')

def green(text: str) -> str:
    # return text wrapped in real ANSI color escapes (green)
    return '\x1b[32m' + str(text) + '\x1b[0m'

def render_progress_bar(pct: float, width: int) -> str:
    """Return a colored progress bar string for percent value 0..100."""
    pct = max(0.0, min(100.0, pct))
    filled = int((pct / 100.0) * width)
    bar = '█' * filled + ' ' * (width - filled)
    return f"{green(bar[:filled])}{bar[filled:]}"


def percent_to_abs(pct: float) -> int:
    """Map 0..100 percent to ABS_MIN..ABS_MAX integer range."""
    return int((max(0.0, min(100.0, pct)) / 100.0) * (ABS_MAX - ABS_MIN) + ABS_MIN)


def compute_adaptive_alpha(base_alpha: float, diff: float) -> float:
    """Return an adapted alpha (0..1) that increases only for sufficiently large changes.

    - base_alpha: the configured baseline alpha (0..1)
    - diff: absolute difference in percent (0..100)

    Adaptive alpha stays at base_alpha for small diffs (<= DIFF_THRESHOLD). For larger diffs
    it scales toward 1.0; ADAPTIVE_SENSITIVITY controls how quickly it approaches 1.0.
    """
    if diff <= DIFF_THRESHOLD:
        return base_alpha
    # normalize remaining range (DIFF_THRESHOLD..100) to 0..1
    adj = (diff - DIFF_THRESHOLD) / (100.0 - DIFF_THRESHOLD)
    scale = min(1.0, adj * ADAPTIVE_SENSITIVITY)
    return base_alpha + (1.0 - base_alpha) * scale


# --------- Main ---------

def main():
    parser = argparse.ArgumentParser(description='ESP32 pedals -> virtual joystick')
    parser.add_argument('--port', '-p', default=None, help='Serial port (auto-detected if omitted)')
    parser.add_argument('--baud', '-b', type=int, default=DEFAULT_BAUD, help=f'Baud rate (default {DEFAULT_BAUD})')
    parser.add_argument('--alpha', type=float, default=ALPHA, help='Base EMA alpha (smoothing) 0..1')
    parser.add_argument('--deadzone', type=float, default=DEADZONE_TOP_PERCENT, help='Top deadzone percent (values >= 100-deadzone snap to 100)')
    args = parser.parse_args()

    alpha = max(0.0, min(1.0, args.alpha))
    dz_top = max(0.0, min(100.0, args.deadzone))
    dz_bottom = max(0.0, DEADZONE_BOTTOM_PERCENT)

    # Detect serial port if user omitted it
    port = args.port if args.port is not None else find_serial_port()

    # Ensure uinput exists (try to modprobe if missing)
    modprobe_uinput()
    if not os.path.exists('/dev/uinput'):
        print('/dev/uinput not found — you may need to run: sudo modprobe uinput')
        print('If running without root, you may also need to adjust uinput permissions or run with sudo.')

    # Capability map for three absolute axes
    cap = {
        ecodes.EV_ABS: [
            (AXES[0], AbsInfo(value=0, min=ABS_MIN, max=ABS_MAX, fuzz=0, flat=0, resolution=0)),
            (AXES[1], AbsInfo(value=0, min=ABS_MIN, max=ABS_MAX, fuzz=0, flat=0, resolution=0)),
            (AXES[2], AbsInfo(value=0, min=ABS_MIN, max=ABS_MAX, fuzz=0, flat=0, resolution=0)),
        ],
        ecodes.EV_KEY: [ecodes.BTN_0],  # <-- Add this line
    }

    try:
        ui = UInput(cap, name='ESP32 Pedals', version=0x3, bustype=ecodes.BUS_USB)
    except PermissionError:
        print('Permission error creating uinput device. Try running with sudo or enable /dev/uinput access for your user.')
        sys.exit(1)
    except Exception as exc:
        print('Failed to create UInput device:', exc)
        sys.exit(1)

    # open serial
    try:
        ser = serial.Serial(port, args.baud, timeout=SERIAL_TIMEOUT)
    except Exception as e:
        print(f'Failed to open serial port {port}: {e}')
        ui.close()
        sys.exit(2)

    smoothed = [0.0, 0.0, 0.0]
    has_value = [False, False, False]
    last_print = 0.0

    print('Starting — press Ctrl-C to quit')
    if os.geteuid() != 0:
        print('Note: running without root. You may need sudo to create uinput device or access the serial port.')
    time.sleep(0.05)

    try:
        while True:
            now = time.time()
            raw_line = ser.readline().decode(errors='ignore').strip()
            parsed = None
            if raw_line:
                try:
                    parsed = ast.literal_eval(raw_line)
                    if not isinstance(parsed, (list, tuple)) or len(parsed) < 3:
                        parsed = None
                except Exception:
                    parsed = None

            if parsed is None:
                values = smoothed[:]  # reuse last smoothed values
            else:
                values = []
                for i in range(3):
                    v = float(parsed[i])

                    # apply bottom deadzone (optional)
                    if v <= dz_bottom:
                        v = 0.0

                    # apply top deadzone (snap very-high values to 100)
                    if v >= 100.0 - dz_top:
                        v = 100.0

                    # smoothing: adaptive EMA that responds faster for large changes
                    if not has_value[i]:
                        smoothed[i] = v
                        has_value[i] = True
                    else:
                        diff = abs(v - smoothed[i])  # percent difference
                        alpha_eff = compute_adaptive_alpha(alpha, diff)
                        smoothed[i] = (alpha_eff * v) + ((1.0 - alpha_eff) * smoothed[i])

                    values.append(smoothed[i])

                # write events to virtual device
                for i, pct in enumerate(values):
                    ui.write(ecodes.EV_ABS, AXES[i], percent_to_abs(pct))
                ui.syn()

            # redraw UI at limited rate
            if now - last_print >= REDRAW_INTERVAL:
                last_print = now
                clear_screen()
                print('ESP32 Pedals — serial:', port, ' baud:', args.baud)
                print()
                if raw_line:
                    print('raw:', raw_line)
                else:
                    print('raw: <no data>')
                print()

                for i, pct in enumerate(values):
                    bar = render_progress_bar(pct, BAR_WIDTH)
                    print(bar)
                    print(f"{LABELS[i]} — {pct:6.2f}%")
                    print()

                sys.stdout.flush()

            time.sleep(0.002)

    except KeyboardInterrupt:
        print('Exiting...')
    finally:
        try:
            ui.close()
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass


if __name__ == '__main__':
    main()
