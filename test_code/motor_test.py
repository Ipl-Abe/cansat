import time
import RPi.GPIO as GPIO
import cv2

cv2.namedWindow('test')

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup(2, GPIO.OUT)
GPIO.setup(3, GPIO.OUT)

pwm1 = GPIO.PWM(2, 50)
pwm2 = GPIO.PWM(3, 50)
pwm1.start(0)
pwm2.start(0)

time.sleep(2)

print("start")


while True:

    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    

    if key == ord('f'):
        print "forward"
        pwm1.ChangeDutyCycle(0)
        pwm2.ChangeDutyCycle(20)
        #GPIO.output(2, 0)
        #GPIO.output(3, 1)
        time.sleep(1)

    if key == ord('b'):
        print "forward"
        #GPIO.output(2, 0)
        #GPIO.output(3, 1)
        time.sleep(1)
        
    if key == ord('s'):
        print "stop"
        pwm1.ChangeDutyCycle(0)
        pwm2.ChangeDutyCycle(0)
        #GPIO.output(2, 0)
        #GPIO.output(3, 0)
        time.sleep(1)


