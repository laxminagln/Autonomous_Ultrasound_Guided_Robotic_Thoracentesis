#!/usr/bin/env python3
"""
Control a Grove I2C Motor Driver v1.3 / L298P from Jetson Orin.

Board defaults:
- I2C address: 0x0f
- Commands based on Seeed Grove_I2C_Motor_Driver library

Usage examples:
    python3 pump_motor_i2c.py --bus 1 --speed 60 --time 5
    python3 pump_motor_i2c.py --bus 7 --speed -50 --time 3
    python3 pump_motor_i2c.py --bus 7 --stop
"""

import time
import argparse
from smbus2 import SMBus

# I2C address
DEFAULT_ADDR = 0x0F

# Commands from Seeed library
MOTOR_SPEED_SET = 0x82
PWM_FREQUENCY_SET = 0x84
DIRECTION_SET = 0xAA

# Motor IDs
MOTOR1 = 1
MOTOR2 = 2

# Direction bit patterns from Seeed library
BOTH_CLOCKWISE = 0x0A
BOTH_ANTICLOCKWISE = 0x05
M1_CW_M2_ACW = 0x06
M1_ACW_M2_CW = 0x09

# PWM frequency presets from Seeed library
F_31372HZ = 0x01
F_3921HZ  = 0x02   # default
F_490HZ   = 0x03
F_122HZ   = 0x04
F_30HZ    = 0x05


class GroveI2CMotorDriver:
    def __init__(self, bus_num: int, address: int = DEFAULT_ADDR):
        self.bus_num = bus_num
        self.address = address
        self.bus = SMBus(bus_num)

        # internal state mirrors official library behavior
        self.speed1 = 0   # 0..255
        self.speed2 = 0   # 0..255
        self.m1_dir = 1   # +1 clockwise, -1 anticlockwise
        self.m2_dir = 1

    def close(self):
        self.bus.close()

    def _write_bytes(self, values):
        self.bus.write_i2c_block_data(self.address, values[0], values[1:])

    def set_frequency(self, freq_code=F_3921HZ):
        # sends 2 bytes like official library
        self._write_bytes([PWM_FREQUENCY_SET, freq_code & 0xFF, (freq_code >> 8) & 0xFF])

    def _send_direction(self):
        if self.m1_dir == 1 and self.m2_dir == 1:
            direction = BOTH_CLOCKWISE
        elif self.m1_dir == -1 and self.m2_dir == -1:
            direction = BOTH_ANTICLOCKWISE
        elif self.m1_dir == 1 and self.m2_dir == -1:
            direction = M1_CW_M2_ACW
        elif self.m1_dir == -1 and self.m2_dir == 1:
            direction = M1_ACW_M2_CW
        else:
            direction = BOTH_CLOCKWISE

        self._write_bytes([DIRECTION_SET, direction, 0x01])

    def _send_speed(self):
        self._write_bytes([MOTOR_SPEED_SET, self.speed1, self.speed2])

    @staticmethod
    def _percent_to_byte(percent: int) -> int:
        percent = max(-100, min(100, percent))
        return int(abs(percent) * 255 / 100)

    def set_motor(self, motor_id: int, percent: int):
        percent = max(-100, min(100, percent))
        pwm = self._percent_to_byte(percent)

        if motor_id == MOTOR1:
            self.m1_dir = 1 if percent >= 0 else -1
            self.speed1 = pwm
        elif motor_id == MOTOR2:
            self.m2_dir = 1 if percent >= 0 else -1
            self.speed2 = pwm
        else:
            raise ValueError("motor_id must be 1 or 2")

        self._send_direction()
        time.sleep(0.01)
        self._send_speed()

    def stop_motor(self, motor_id: int):
        if motor_id == MOTOR1:
            self.speed1 = 0
        elif motor_id == MOTOR2:
            self.speed2 = 0
        else:
            raise ValueError("motor_id must be 1 or 2")

        self._send_speed()

    def stop_all(self):
        self.speed1 = 0
        self.speed2 = 0
        self._send_speed()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bus", type=int, required=True, help="I2C bus number, e.g. 1 or 7")
    parser.add_argument("--addr", type=lambda x: int(x, 0), default=DEFAULT_ADDR, help="I2C address, default 0x0f")
    parser.add_argument("--motor", type=int, default=1, choices=[1, 2], help="Motor channel: 1 or 2")
    parser.add_argument("--speed", type=int, default=60, help="Speed percent from -100 to 100")
    parser.add_argument("--time", type=float, default=5.0, help="Run time in seconds")
    parser.add_argument("--stop", action="store_true", help="Stop motor and exit")
    args = parser.parse_args()

    drv = GroveI2CMotorDriver(args.bus, args.addr)

    try:
        # Start with default frequency used by official library
        drv.set_frequency(F_3921HZ)
        time.sleep(0.05)

        if args.stop:
            drv.stop_motor(args.motor)
            print(f"Stopped motor {args.motor}")
            return

        print(f"Running motor {args.motor} at {args.speed}% for {args.time} s")
        drv.set_motor(args.motor, args.speed)
        time.sleep(args.time)
        drv.stop_motor(args.motor)
        print("Done, motor stopped.")

    finally:
        drv.close()


if __name__ == "__main__":
    main()