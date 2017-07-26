import time
import RPi.GPIO as GPIO
import cv2

cv2.namedWindow('test')

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)

pwm = GPIO.PWM(16, 50)
pwm.start(0)

speed = 100

time.sleep(2)

print("start")


while True:

    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    

    if key == ord('f'):
        print "forward"
        pwm.ChangeDutyCycle(speed)
        GPIO.output(20, 0)
        GPIO.output(21, 1)
        time.sleep(1)
        
    if key == ord('s'):
        print "stop"
        pwm.ChangeDutyCycle(0)
        GPIO.output(20, 0)
        GPIO.output(21, 0)
        time.sleep(1)

    if key == ord('e'):
        print "end"
        pwm.stop()
        GPIO.cleanup()
        break
