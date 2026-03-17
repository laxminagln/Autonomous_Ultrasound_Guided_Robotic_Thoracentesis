import time
import board
import busio
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(board.SCL, board.SDA)

pca = PCA9685(i2c)
pca.frequency = 50

servo = pca.channels[8]

def set_pulse(us):
    pulse_length = int(us * 4096 / 20000)
    servo.duty_cycle = pulse_length << 4

try:
    while True:

        print("0 degrees")
        set_pulse(1000)
        time.sleep(2)

        print("90 degrees")
        set_pulse(1500)
        time.sleep(2)

        print("180 degrees")
        set_pulse(2000)
        time.sleep(2)

except KeyboardInterrupt:
    pass
