import Motor
import cv2
import time
import RPi.GPIO as GPIO

cv2.namedWindow('test')


motor = Motor.Motor(7, 25, 23, 24)
speed = 100
flag = 0
motor.set_speed(speed)
start = time.clock()

while True:


    '''
    n_time = time.clock() - start
    
    if n_time > 0.2:
        print str(flag) + ", " + str(speed)
        if speed > 80 and flag == 0:
            flag = 1
        if speed < 10 and flag == 1:
            flag = 0
                
        if flag == 0:
            speed = speed + 5
        if flag == 1:
            speed = speed - 5

        print str(flag) + ", " + str(speed)
        motor.set_speed(speed)
        start = time.clock()
    '''

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

    if key == ord('f'):
        print "forward"
        motor.move_forward()

    if key == ord("b"):
        print "backward"
        motor.move_backward()

    if key == ord("r"):
        print "right"
        motor.turn_right()

    if key == ord("l"):
        print "left"
        motor.turn_left()

    if key == ord("s"):
        print "stop"
        motor.stop()

    if key == ord("e"):
        print "end"
        motor.finish()
        GPIO.cleanup()
        break
