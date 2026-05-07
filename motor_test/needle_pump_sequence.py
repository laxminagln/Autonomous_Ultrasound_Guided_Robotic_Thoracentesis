#!/usr/bin/env python3
"""Thoracentesis sequence on Jetson Orin using two different I2C buses.

Wiring intent:
- Motor driver on Jetson header pins 3/5  -> I2C bus 7
- PCA9685 servo driver on Jetson header pins 27/28 -> alternate I2C bus

Sequence:
1. Move needle DOWN first
2. Run peristaltic pump for 30 s using M2
3. Stop pump
4. Move needle UP
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
MOTOR_I2C_BUS = 7
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
    def __init__(self, bus_num=MOTOR_I2C_BUS, addr=I2C_ADDR):
        self.bus_num = bus_num
        self.addr = addr
        self.bus = SMBus(bus_num)
        self.speed1 = 0
        self.speed2 = 0
        self.dir1 = 1
        self.dir2 = 1
        self._last_direction = None

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass

    def _reopen_bus(self):
        try:
            self.bus.close()
        except Exception:
            pass
        time.sleep(0.05)
        self.bus = SMBus(self.bus_num)

    def _write(self, cmd, data, retries=3):
        last_exc = None
        for attempt in range(retries):
            try:
                self.bus.write_i2c_block_data(self.addr, cmd, data)
                return
            except Exception as exc:
                last_exc = exc
                print(f"I2C write failed for cmd 0x{cmd:02X}: {exc}. Retrying...")
                self._reopen_bus()
                time.sleep(0.1 * (attempt + 1))
        raise last_exc

    def set_pwm_frequency(self, freq=F_3921HZ):
        self._write(PWM_FREQUENCY_SET, [freq, 0x00])
        time.sleep(0.05)

    def _direction_code(self):
        if self.dir1 == 1 and self.dir2 == 1:
            return BOTH_CLOCKWISE
        if self.dir1 == -1 and self.dir2 == -1:
            return BOTH_ANTICLOCKWISE
        if self.dir1 == 1 and self.dir2 == -1:
            return M1_CW_M2_ACW
        return M1_ACW_M2_CW

    def _send_direction(self, force=False):
        direction = self._direction_code()
        if force or direction != self._last_direction:
            self._write(DIRECTION_SET, [direction, 0x01])
            self._last_direction = direction
            time.sleep(0.01)

    def _send_speed(self):
        self._write(MOTOR_SPEED_SET, [self.speed1, self.speed2])

    def set_motor1(self, percent, force_direction=False):
        percent = max(-100, min(100, percent))
        self.dir1 = 1 if percent >= 0 else -1
        self.speed1 = int(abs(percent) * 255 / 100)
        self._send_direction(force=force_direction)
        self._send_speed()

    def set_motor2(self, percent, force_direction=False):
        percent = max(-100, min(100, percent))
        self.dir2 = 1 if percent >= 0 else -1
        self.speed2 = int(abs(percent) * 255 / 100)
        self._send_direction(force=force_direction)
        self._send_speed()

    def stop_motor1(self):
        self.speed1 = 0
        self._send_speed()

    def stop_motor2(self):
        self.speed2 = 0
        self._send_speed()

    def stop_all(self):
        self.speed1 = 0
        self.speed2 = 0
        self._send_speed()


# =========================
# PCA9685 Servo Control
# =========================
def make_alt_i2c_for_pins_27_28():
    """Create the alternate I2C bus used by Jetson header pins 27/28."""
    scl = getattr(board, "SCL_1", None)
    sda = getattr(board, "SDA_1", None)
    if scl is None or sda is None:
        raise RuntimeError(
            "Alternate I2C pins 27/28 are not exposed as board.SCL_1 / board.SDA_1 on this system. "
            "Run: python3 -c \"import board; print(dir(board))\" and confirm the alternate I2C names."
        )
    return busio.I2C(scl, sda)


class NeedleServo:
    def __init__(self, channel=8, frequency=50, address=0x50):
        self.i2c = make_alt_i2c_for_pins_27_28()
        self.pca = PCA9685(self.i2c, address=address)
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
    # down_pulse,
    servo_move_time,
    settle_time_after_down,
    pump_speed,
    pump_run_time,
    release_servo_at_end,
):
    servo = None
    pump = None

    try:
        print("Initializing pump on I2C pins 3/5 (bus 7) and servo driver on I2C pins 27/28...")
        servo = NeedleServo(channel=8, frequency=50, address=0x50)
        pump = GroveMotorDriver()
        pump.set_pwm_frequency(F_3921HZ)

        servo.set_pulse_us(up_pulse)
        time.sleep(1.0)

        # STEP 1: needle goes DOWN first
        # print(f"Moving needle DOWN ({down_pulse} us)")
        # servo.set_pulse_us(down_pulse)
        # time.sleep(servo_move_time)

        if settle_time_after_down > 0:
            print(f"Needle reached down position. Waiting {settle_time_after_down:.1f} s...")
            time.sleep(settle_time_after_down)

        # STEP 2: run pump on M2
        # STEP 2: run pump on M2 in opposite direction for 5 s
        print(f"Running peristaltic pump on M2 in opposite direction at {-pump_speed}% for 5.0 s...")
        pump.set_motor2(pump_speed, force_direction=True)
        time.sleep(3.0)

        # Then run in normal/current direction for 20 s
        print(f"Running peristaltic pump on M2 in normal direction at {pump_speed}% for 20.0 s...")
        pump.set_motor2(-pump_speed, force_direction=True)
        time.sleep(20.0)

        # STEP 3: stop pump on M2
        print("Stopping peristaltic pump on M2...")
        try:
            pump.stop_motor2()
        except Exception as exc:
            print(f"Warning: failed to send stop command cleanly: {exc}")
        time.sleep(0.5)

        # STEP 4: needle goes UP
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
        description="Run thoracentesis sequence: needle down -> pump on M2 for 10s -> pump off -> needle up"
    )
    # parser.add_argument(
    #     "--down-pulse",
    #     type=int,
    #     default=1250,
    #     help="Servo pulse (us) for needle DOWN position"
    # )
    parser.add_argument(
        "--up-pulse",
        type=int,
        default=2500,
        help="Servo pulse (us) for needle UP position"
    )
    parser.add_argument(
        "--servo-move-time",
        type=float,
        default=4.0,
        help="Time in seconds to allow servo motion"
    )
    parser.add_argument(
        "--settle-time-after-down",
        type=float,
        default=1.0,
        help="Wait time after needle reaches down position before pump starts"
    )
    parser.add_argument(
        "--pump-speed",
        type=int,
        default=100,
        help="Pump running speed in percent (-100 to 100)"
    )
    parser.add_argument(
        "--pump-time",
        type=float,
        default=20.0,
        help="Pump run time in seconds"
    )
    parser.add_argument(
        "--no-release-servo",
        action="store_true",
        help="Keep servo driven at the end instead of releasing PWM"
    )
    args = parser.parse_args()

    run_thoracentesis_sequence(
        up_pulse=args.up_pulse,
        # down_pulse=args.down_pulse,
        servo_move_time=args.servo_move_time,
        settle_time_after_down=args.settle_time_after_down,
        pump_speed=args.pump_speed,
        pump_run_time=args.pump_time,
        release_servo_at_end=False,
    )


if __name__ == "__main__":
    main()