#!/usr/bin/env python3
import time
import argparse
from smbus2 import SMBus

# Your detected settings
I2C_BUS = 7
I2C_ADDR = 0x0F

# Grove I2C Motor Driver commands
MOTOR_SPEED_SET = 0x82
PWM_FREQUENCY_SET = 0x84
DIRECTION_SET = 0xAA

# PWM frequency options
F_31372HZ = 0x01
F_3921HZ  = 0x02
F_490HZ   = 0x03
F_122HZ   = 0x04
F_30HZ    = 0x05

# Direction values
BOTH_CLOCKWISE = 0x0A
BOTH_ANTICLOCKWISE = 0x05
M1_CW_M2_ACW = 0x06
M1_ACW_M2_CW = 0x09


class GroveMotorDriver:
    def __init__(self, bus_num=I2C_BUS, addr=I2C_ADDR):
        self.bus = SMBus(bus_num)
        self.addr = addr
        self.speed1 = 0
        self.speed2 = 0
        self.dir1 = 1
        self.dir2 = 1

    def close(self):
        self.bus.close()

    def _write(self, cmd, data):
        self.bus.write_i2c_block_data(self.addr, cmd, data)

    def set_pwm_frequency(self, freq=F_3921HZ):
        self._write(PWM_FREQUENCY_SET, [freq, 0x00])
        time.sleep(0.05)

    def _send_direction(self):
        if self.dir1 == 1 and self.dir2 == 1:
            direction = BOTH_CLOCKWISE
        elif self.dir1 == -1 and self.dir2 == -1:
            direction = BOTH_ANTICLOCKWISE
        elif self.dir1 == 1 and self.dir2 == -1:
            direction = M1_CW_M2_ACW
        else:
            direction = M1_ACW_M2_CW

        self._write(DIRECTION_SET, [direction, 0x01])
        time.sleep(0.01)

    def _send_speed(self):
        self._write(MOTOR_SPEED_SET, [self.speed1, self.speed2])

    def set_motor1(self, percent):
        percent = max(-100, min(100, percent))
        self.dir1 = 1 if percent >= 0 else -1
        self.speed1 = int(abs(percent) * 255 / 100)
        self._send_direction()
        self._send_speed()

    def stop_motor1(self):
        self.speed1 = 0
        self._send_speed()

    def stop_all(self):
        self.speed1 = 0
        self.speed2 = 0
        self._send_speed()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=int, default=30, help="Motor speed from -100 to 100")
    parser.add_argument("--time", type=float, default=3.0, help="Run time in seconds")
    args = parser.parse_args()

    drv = GroveMotorDriver()

    try:
        drv.set_pwm_frequency(F_3921HZ)

        print(f"Running motor on OUT1/OUT2 at {args.speed}% for {args.time} seconds")
        drv.set_motor1(args.speed)
        time.sleep(args.time)

        drv.stop_motor1()
        print("Motor stopped")

    finally:
        drv.close()


if __name__ == "__main__":
    main()