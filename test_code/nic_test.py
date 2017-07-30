import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(2, GPIO.OUT)

print "start"

print "cut"
GPIO.output(2, GPIO.HIGH)
time.sleep(10)

print "finish"
GPIO.output(2, GPIO.LOW)

GPIO.cleanup()
