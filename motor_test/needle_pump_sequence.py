#!/usr/bin/env python3
"""Control thoracentesis needle servo + peristaltic pump sequence on Jetson Orin.

Sequence:
1. Move servo down (needle insertion)
2. Run pump for configured time
3. Stop pump
4. Move servo up (needle retraction)

Based on the user's working Grove I2C motor driver script and PCA9685 servo test.
"""

import time
import argparse
from smbus2 import SMBus
import board
import busio
from adafruit_pca9685 import PCA9685

# =========================
# Grove I2C Motor Driver
# =========================
I2C_BUS = 7
I2C_ADDR = 0x0F

MOTOR_SPEED_SET = 0x82
PWM_FREQUENCY_SET = 0x84
DIRECTION_SET = 0xAA

F_31372HZ = 0x01
F_3921HZ = 0x02
F_490HZ = 0x03
F_122HZ = 0x04
F_30HZ = 0x05

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


# =========================
# PCA9685 Servo Control
# =========================
class NeedleServo:
    def __init__(self, channel=8, frequency=50):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = frequency
        self.servo = self.pca.channels[channel]

    def set_pulse_us(self, us):
        pulse_length = int(us * 4096 / 20000)  # 20 ms period at 50 Hz
        self.servo.duty_cycle = pulse_length << 4

    def release(self):
        self.servo.duty_cycle = 0

    def close(self):
        try:
            self.release()
        finally:
            self.pca.deinit()


# =========================
# Combined Sequence
# =========================
def run_thoracentesis_sequence(
    up_pulse,
    down_pulse,
    servo_move_time,
    pump_speed,
    pump_run_time,
    pump_start_boost,
    pump_boost_time,
    settle_time,
    release_servo_at_end,
):
    servo = None
    pump = None

    try:
        print("Initializing servo and pump...")
        servo = NeedleServo(channel=8, frequency=50)
        pump = GroveMotorDriver()
        pump.set_pwm_frequency(F_3921HZ)

        # Ensure starting position is UP
        print(f"Moving needle to UP position ({up_pulse} us)")
        servo.set_pulse_us(up_pulse)
        time.sleep(servo_move_time)

        # Needle down
        print(f"Moving needle DOWN ({down_pulse} us)")
        servo.set_pulse_us(down_pulse)
        time.sleep(servo_move_time)

        if settle_time > 0:
            print(f"Waiting {settle_time:.1f} s after insertion...")
            time.sleep(settle_time)

        # Pump on
        print("Starting peristaltic pump...")
        if pump_start_boost > 0 and pump_boost_time > 0:
            print(f"Pump boost: {pump_start_boost}% for {pump_boost_time:.2f} s")
            pump.set_motor1(pump_start_boost)
            time.sleep(pump_boost_time)

        print(f"Pump run: {pump_speed}% for {pump_run_time:.1f} s")
        pump.set_motor1(pump_speed)
        time.sleep(pump_run_time)

        # Pump off
        print("Stopping peristaltic pump...")
        pump.stop_motor1()
        time.sleep(0.5)

        # Needle up
        print(f"Moving needle UP ({up_pulse} us)")
        servo.set_pulse_us(up_pulse)
        time.sleep(servo_move_time)

        if release_servo_at_end:
            print("Releasing servo signal.")
            servo.release()

        print("Sequence complete.")

    except KeyboardInterrupt:
        print("\nInterrupted. Stopping pump and returning needle up...")
        if pump is not None:
            try:
                pump.stop_all()
            except Exception:
                pass
        if servo is not None:
            try:
                servo.set_pulse_us(up_pulse)
                time.sleep(servo_move_time)
                if release_servo_at_end:
                    servo.release()
            except Exception:
                pass
        raise

    finally:
        if pump is not None:
            try:
                pump.stop_all()
            except Exception:
                pass
            try:
                pump.close()
            except Exception:
                pass
        if servo is not None:
            try:
                servo.close()
            except Exception:
                pass


def main():
    parser = argparse.ArgumentParser(
        description="Run thoracentesis sequence: needle down -> pump on -> pump off -> needle up"
    )
    parser.add_argument("--up-pulse", type=int, default=1000,
                        help="Servo pulse (us) for needle UP position")
    parser.add_argument("--down-pulse", type=int, default=2000,
                        help="Servo pulse (us) for needle DOWN position")
    parser.add_argument("--servo-move-time", type=float, default=2.0,
                        help="Time in seconds to allow servo motion")
    parser.add_argument("--pump-speed", type=int, default=65,
                        help="Pump running speed in percent (-100 to 100)")
    parser.add_argument("--pump-time", type=float, default=30.0,
                        help="Pump run time in seconds")
    parser.add_argument("--pump-start-boost", type=int, default=100,
                        help="Short startup boost speed in percent to avoid stalling")
    parser.add_argument("--pump-boost-time", type=float, default=0.8,
                        help="Boost duration in seconds")
    parser.add_argument("--settle-time", type=float, default=0.5,
                        help="Wait time after needle reaches down position before starting pump")
    parser.add_argument("--no-release-servo", action="store_true",
                        help="Keep servo driven at the end instead of releasing PWM")
    args = parser.parse_args()

    run_thoracentesis_sequence(
        up_pulse=args.up_pulse,
        down_pulse=args.down_pulse,
        servo_move_time=args.servo_move_time,
        pump_speed=args.pump_speed,
        pump_run_time=args.pump_time,
        pump_start_boost=args.pump_start_boost,
        pump_boost_time=args.pump_boost_time,
        settle_time=args.settle_time,
        release_servo_at_end=not args.no_release_servo,
    )


if __name__ == "__main__":
    main()