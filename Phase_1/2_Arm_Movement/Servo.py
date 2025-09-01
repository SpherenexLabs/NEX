# File: pca_servos_seq.py
# ─────────────────────────────────────────────────────────────────────────────
# SUMMARY
#   • Raspberry Pi 5 controls servos via PCA9685 (I2C address 0x40) at 50 Hz.
#   • Active channels: CH0–CH4 and CH6 (CH5 skipped).
#   • Each servo is swept 0→180→0 sequentially, then returns to center (90°).
#   • Power Note: Servos must use a separate 5–6 V supply on V+; GNDs common.
#
# WIRING (TABLE)
# ┌────────────────────┬──────────────┬───────────────────────────────────────┐
# │ Signal / Purpose   │ PCA9685 Pin  │ Connect To                           │
# ├────────────────────┼──────────────┼───────────────────────────────────────┤
# │ Logic VCC          │ VCC          │ Raspberry Pi 3.3 V                   │
# │ Logic GND          │ GND          │ Raspberry Pi GND (and servo GND)     │
# │ I2C SDA            │ SDA          │ Pi GPIO2 / SDA1 (pin 3)              │
# │ I2C SCL            │ SCL          │ Pi GPIO3 / SCL1 (pin 5)              │
# │ Servo Power (+)    │ V+           │ 5–6 V external supply (NOT Pi 5 V)   │
# │ Servo Power (−)    │ GND          │ External supply GND (tie to Pi GND)  │
# │ OE (optional)      │ OE           │ GND (always enabled) or a Pi GPIO    │
# │ Servo on CH0       │ CH0 Signal   │ Servo signal (yellow/white)          │
# │ Servo on CH1       │ CH1 Signal   │ Servo signal                          │
# │ Servo on CH2       │ CH2 Signal   │ Servo signal                          │
# │ Servo on CH3       │ CH3 Signal   │ Servo signal                          │
# │ Servo on CH4       │ CH4 Signal   │ Servo signal                          │
# │ Servo on CH6       │ CH6 Signal   │ Servo signal                          │
# └────────────────────┴──────────────┴───────────────────────────────────────┘
#
# Tips:
#   • Put a bulk capacitor (e.g., 1000 µF, ≥10 V) across V+ and GND near the PCA9685.
#   • MG9xx/DS3225-class servos can draw >1 A each at stall; size the PSU accordingly.
#   • Install libs on Pi:  pip3 install adafruit-circuitpython-pca9685 adafruit-blinka
#   • Enable I2C on Raspberry Pi (raspi-config or Raspberry Pi Imager advanced options).
# ─────────────────────────────────────────────────────────────────────────────

import time
import board, busio
from adafruit_pca9685 import PCA9685

# --- Servo settings ---
SERVO_FREQ = 50.0          # Hz
SERVO_MIN_US = 600         # adjust per servo if needed
SERVO_MAX_US = 2400

# Active servo channels (skip CH5)
SERVO_CHANNELS = [0, 1, 2, 3, 4, 6]

def us_to_duty(pulse_us, freq=SERVO_FREQ):
    period_us = 1_000_000.0 / freq
    duty = int((pulse_us / period_us) * 0xFFFF)
    return max(0, min(0xFFFF, duty))

def deg_to_us(deg):
    deg = max(0, min(180, deg))
    return SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (deg / 180.0)

# Init I2C + PCA9685 (default address 0x40)
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c, address=0x40)
pca.frequency = int(SERVO_FREQ)

def write_servo(ch, deg):
    pulse_us = deg_to_us(deg)
    pca.channels[ch].duty_cycle = us_to_duty(pulse_us)

# Center all active servos
for ch in SERVO_CHANNELS:
    write_servo(ch, 90)
time.sleep(0.5)

print("Sequential sweep (CH0–CH4, CH6)")
try:
    while True:
        for ch in SERVO_CHANNELS:
            # Sweep this channel 0 -> 180 -> 0
            for a in range(0, 181, 3):
                write_servo(ch, a)
                time.sleep(0.01)
            for a in range(180, -1, -3):
                write_servo(ch, a)
                time.sleep(0.01)
            write_servo(ch, 90)   # return to center
            time.sleep(0.6)
except KeyboardInterrupt:
    pass
finally:
    # Release safely
    for ch in SERVO_CHANNELS:
        write_servo(ch, 90)
        time.sleep(0.05)
        pca.channels[ch].duty_cycle = 0
    pca.deinit()
    print("\nPCA9685 released.")
