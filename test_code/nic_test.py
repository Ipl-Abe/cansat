import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(2, GPIO.OUT)

nic = GPIO.PWM(2, 100)
nic.start(0)

print "start"

print "cut"
nic.ChangeDutyCycle(100)
time.sleep(3)

print "finish"
nic.ChangeDutyCycle(0)

GPIO.cleanup()
