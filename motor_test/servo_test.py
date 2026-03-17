import time
import Jetson.GPIO as GPIO

SERVO_PIN = 15


GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(0)

def angle_to_duty(angle):
    angle = max(0, min(180, angle))
    return 2.5 + (angle/180.0)*10.0

try:
    print("Moving to 0 degrees")
    pwm.ChangeDutyCycle(angle_to_duty(0))
    time.sleep(2.0)

    print("Moving to 90 degrees")
    pwm.ChangeDutyCycle(angle_to_duty(90))
    time.sleep(2.0)

    print("Moving to 180 degrees")
    pwm.ChangeDutyCycle(angle_to_duty(180))
    time.sleep(2.0)

    print("Returning to 90 degrees")
    pwm.ChangeDutyCycle(angle_to_duty(90))
    time.sleep(2.0)

    print("Returning to 0 degrees")
    pwm.ChangeDutyCycle(angle_to_duty(0))
    time.sleep(2.0)

    pwm.ChangeDutyCycle(0)
    time.sleep(1.0)

except KeyboardInterrupt:
    pass

finally:
    pwm.stop()
    GPIO.cleanup()