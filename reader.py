# ESP32 + MicroPython
# 3x MPU6050 on a TCA9548A I2C multiplexer, normalized 0..100
# - Uses first reading per sensor as initial MAX
# - Tracks MIN/MAX with 1 s "debounce" before accepting new extremes
# - Inverts the 3rd sensor (index 2), like your original

import math
import struct
import utime
from machine import I2C, Pin

# ---------- CONFIG ----------
MUX_ADDR      = 0x70
MUX_CHANNELS  = [0, 1, 2]   # TCA9548A channels used
MPU_ADDR      = 0x68
I2C_ID        = 0           # 0 or 1 on ESP32 depending on your board
I2C_SCL_PIN   = 22          # change if needed
I2C_SDA_PIN   = 21          # change if needed
I2C_FREQ_HZ   = 400_000     # 400 kHz
DEBOUNCE_MS   = 1000        # hold value for 1s before accepting new min/max
LOOP_DELAY_MS = 10          # ~100 Hz loop

# ---------- I2C SETUP ----------
i2c = I2C(I2C_ID, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ_HZ)

# ---------- TCA9548A ----------
def select_mux_channel(channel: int):
    # Write a single byte with the channel bit set
    i2c.writeto(MUX_ADDR, bytes([1 << channel]))
    utime.sleep_ms(10)  # allow switching time

# ---------- Minimal MPU6050 driver ----------
class MPU6050:
    def __init__(self, i2c_obj, addr=MPU_ADDR):
        self.i2c = i2c_obj
        self.addr = addr
        self._init_device()

    def _init_device(self):
        # Wake up device (PWR_MGMT_1 = 0x6B -> 0x00)
        try:
            self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')
            utime.sleep_ms(100)
            # Accel config ±2g (ACCEL_CONFIG = 0x1C -> 0x00)
            self.i2c.writeto_mem(self.addr, 0x1C, b'\x00')
            utime.sleep_ms(10)
        except OSError:
            # If init happens with the wrong mux channel selected,
            # we'll just hit it again on first successful select/read.
            pass

    def get_accel_data(self):
        # Returns dict with keys 'x','y','z' in g units (±2g scale => 16384 LSB/g)
        data = self.i2c.readfrom_mem(self.addr, 0x3B, 6)
        ax, ay, az = struct.unpack('>hhh', data)
        scale = 16384.0
        return {'x': ax / scale, 'y': ay / scale, 'z': az / scale}

# ---------- Helpers ----------
def safe_read(read_fn, retries=3, delay_ms=5):
    for _ in range(retries):
        try:
            return read_fn()
        except OSError:
            utime.sleep_ms(delay_ms)
    return None

def get_pitch(accel):
    # Pitch (deg) from accelerometer only
    ax = accel['x']
    ay = accel['y']
    az = accel['z']
    return math.degrees(math.atan2(ay, math.sqrt(ax*ax + az*az)))

def clamp(val, lo, hi):
    return lo if val < lo else hi if val > hi else val

# ---------- Create sensor objects (one per mux channel) ----------
sensors = []
for ch in MUX_CHANNELS:
    select_mux_channel(ch)
    sensors.append(MPU6050(i2c, MPU_ADDR))

# ---------- Initialize min/max from first reading ----------
min_vals = []
max_vals = []
for idx, ch in enumerate(MUX_CHANNELS):
    select_mux_channel(ch)
    accel = safe_read(lambda: sensors[idx].get_accel_data())
    # If first read fails, try once more after a brief pause
    if accel is None:
        utime.sleep_ms(50)
        accel = safe_read(lambda: sensors[idx].get_accel_data())
    # If still None, fall back to a neutral 0 deg (won't matter after first good read)
    initial = get_pitch(accel) if accel is not None else 0.0
    if idx in [0,2]:
        initial = -initial  # invert third sensor
    min_vals.append(initial)
    max_vals.append(initial)

# Track stability timers for min/max using ticks_ms (wrap-safe)
min_stable_ms = [None, None, None]
max_stable_ms = [None, None, None]

# ---------- Main loop ----------
while True:
    normalized_positions = []
    now = utime.ticks_ms()

    for idx, ch in enumerate(MUX_CHANNELS):
        select_mux_channel(ch)

        accel = safe_read(lambda: sensors[idx].get_accel_data())
        if accel is None:
            current = max_vals[idx]  # fallback to startup value if read fails
        else:
            current = get_pitch(accel)
            if idx in [0,2]:
                current = -current  # invert third sensor

        # ----- Debounced MIN update -----
        if current < min_vals[idx]:
            if min_stable_ms[idx] is None:
                min_stable_ms[idx] = now
            elif utime.ticks_diff(now, min_stable_ms[idx]) >= DEBOUNCE_MS:
                min_vals[idx] = current
                min_stable_ms[idx] = None
        else:
            min_stable_ms[idx] = None

        # ----- Debounced MAX update -----
        if current > max_vals[idx]:
            if max_stable_ms[idx] is None:
                max_stable_ms[idx] = now
            elif utime.ticks_diff(now, max_stable_ms[idx]) >= DEBOUNCE_MS:
                max_vals[idx] = current
                max_stable_ms[idx] = None
        else:
            max_stable_ms[idx] = None

        # ----- Normalize to 0..100 -----
        diff = max_vals[idx] - min_vals[idx]
        if diff == 0:
            norm = 100.0
        else:
            norm = ((current - min_vals[idx]) / diff) * 100.0
        norm = clamp(norm, 0.0, 100.0)

        normalized_positions.append(round(norm, 2))

    print(normalized_positions)
    utime.sleep_ms(LOOP_DELAY_MS)
