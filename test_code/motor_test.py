import time
import RPi.GPIO as GPIO
import cv2

cv2.namedWindow('test')

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)

GPIO.setup(18, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)

GPIO.setup(13, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)

pwmR = GPIO.PWM(21, 50)
pwmR.start(50)
pwmL = GPIO.PWM(18, 50)
pwmL.start(50)

time.sleep(2)

print("start")


while True:

    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    

    if key == ord('f'):
        print "forward"
        GPIO.output(16, 0)
        GPIO.output(20, 1)
        GPIO.output(23, 0)
        GPIO.output(24, 1)
        time.sleep(1)

    if key == ord('b'):
        print "forward"
        GPIO.output(16, 0)
        GPIO.output(20, 1)
        GPIO.output(23, 1)
        GPIO.output(24, 0)
        time.sleep(1)
        
    if key == ord('s'):
        print "stop"
        GPIO.output(16, 0)
        GPIO.output(20, 0)
        GPIO.output(23, 0)
        GPIO.output(24, 0)
        time.sleep(1)


pwmR.stop()
pwmL.stop()
