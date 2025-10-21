# main.py  (on ESP32)
from machine import Pin, I2C
import time

I2C_SCL = 22
I2C_SDA = 21
MUX_ADDR = 0x70
GYRO_ADDR = 0x68
CHANNELS = (0, 1, 2)

def make_i2c():
    for bus in (1, 0):
        try:
            i2c = I2C(bus, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=100000)
            if MUX_ADDR in i2c.scan():
                print("I2C bus", bus, "ready")
                return i2c
        except Exception:
            pass
    raise RuntimeError("Mux 0x70 not found")

i2c = make_i2c()

def mux_select(channel):
    i2c.writeto(MUX_ADDR, bytes([1 << channel]))
    time.sleep_ms(5)

def gyro_init():
    i2c.writeto_mem(GYRO_ADDR, 0x6B, b'\x00')
    time.sleep_ms(10)

def read_gyro():
    data = i2c.readfrom_mem(GYRO_ADDR, 0x43, 6)
    gx = int.from_bytes(data[0:2], 'big', True)
    gy = int.from_bytes(data[2:4], 'big', True)
    gz = int.from_bytes(data[4:6], 'big', True)
    return gx, gy, gz

print("Starting gyro stream...")
time.sleep(1)

while True:
    readings = []
    for ch in CHANNELS:
        try:
            mux_select(ch)
            gyro_init()
            gx, gy, gz = read_gyro()
            readings.append(f"{ch}:{gx},{gy},{gz}")
        except OSError:
            readings.append(f"{ch}:ERR")
    print("|".join(readings))  # one line per update
    time.sleep(0.1)  # adjust rate (0.1 s = 10 Hz)
