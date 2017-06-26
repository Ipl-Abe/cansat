import time
import RPi.GPIO as GPIO
import cv2

cv2.namedWindow('test')

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

GPIO.setup(8, GPIO.OUT)
GPIO.setup(10, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)

pwmR = GPIO.PWM(16, 50)
pwmR.start(100)

time.sleep(2)

print("start")

motor_start = time.perf_counter()


while True:
    motor_time = time.perf_counter() - motor_start

    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    

    if key == ord('f'):
        print("forward")
        GPIO.output(8, 1)
        GPIO.output(10, 0)
        time.sleep(1)

    if key == ord('s'):
        print("stop")
        GPIO.output(8, 0)
        GPIO.output(10, 0)
        time.sleep(1)


pwmR.stop()

