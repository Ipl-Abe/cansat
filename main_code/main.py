# -*- coding:utf-8 -*-

import time
import picamera
import RPi.GPIO as GPIO
import spidev
import cv2
import numpy as np
import Motor
import GPSMode
import ImageProcMode
from gps import *

f = open('record.txt', 'w')

# target GPS data
target_x = 139.938965
target_y = 37.52241

'''
Initialize
'''
stuck_start = time.clock()
    
motor = Motor.Motor(7, 25, 23, 24)
gps_mode = GPSMode.GPSMode()
img_mode = ImageProcMode.ImageProcMode()

GPIO.setmode(GPIO.BCM)
channels = [22, 27, 17, 4]
GPIO.setup(channels, GPIO.IN)
compass = [0, 0, 0, 0]

session = gps()
session.stream(WATCH_ENABLE|WATCH_NEWSTYLE)

spi = spidev.SpiDev()
spi.open(0, 0)

GPIO.setmode(GPIO.BCM)
GPIO.setup(2, GPIO.OUT)

# 巻き取り
GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
pwm = GPIO.PWM(16, 100)
pwm.start(0)

cv2.namedWindow('original image')
cv2.namedWindow('binary image')

def read_compass():
    for i in range(4):
        compass[i] = GPIO.input(channels[i])

    c = 0
    if compass[0] == 1: c = c + 1
    if compass[1] == 1: c = c + 2
    if compass[2] == 1: c = c + 4
    if compass[3] == 1: c = c + 8
    
    if c == 0: return 8
    elif c == 1: return 11
    elif c == 2: return 3
    elif c == 3: return 0
    elif c == 4: return 7
    elif c == 5: return 12
    elif c == 6: return 4
    elif c == 7: return 15
    elif c == 8: return 9
    elif c == 9: return 10
    elif c == 10: return 2
    elif c == 11: return 1
    elif c == 12: return 6
    elif c == 13: return 13
    elif c == 14: return 5
    else: return 14

def read_gps():
    x = 0
    y = 0
    report = session.next()
    if report.keys()[0] == 'epx':
            x = float(report['lon'])
            y = float(report['lat'])
    return x, y

def read_analogData(channel):
        r = spi.xfer2([1, (2 + channel) << 6, 0])
        adc_out = ((r[1]&31) << 6) + (r[2] >> 2)
        return adc_out

def read_ultrasonic():
        analog = read_analogData(0)
        volts = analog * 5.0 / 1024
        inches = volts / 0.0098
        return inches * 2.54     

def capture_image(camera, flag):
    camera.capture('original_image.jpg')
    return cv2.imread('original_image.jpg', flag)

def standby(camera):
    img = capture_image(camera, 0)
    cv2.imshow('original image', img)
    record = str(np.average(img))
    print record
    f.write(record + "\n")
    if np.average(img) > 30:
        return False
    else:
        return True

def falling():

    for i in range(120):
        time.sleep(1)
        record = str(i+1)
        # 最初の30秒間は焼き切り、その後は10秒に1回は焼き切り
        if i < 30 or i%10 == 0:
            GPIO.output(2, GPIO.HIGH)
            record = record + ": cutting"
        else:
            GPIO.output(2, GPIO.LOW)

        # 20秒間焼き切った後から巻き取り開始
        if i > 20:
            pwm.ChangeDutyCycle(100)
            GPIO.output(20, 0)
            GPIO.output(21, 1)
            record = record + ": winding"

        print record

    pwm.ChangeDutyCycle(0)
    pwm.stop()
    
def running(camera, mode):
    global stuck_start
    
    # 停止
    key = cv2.waitKey(1)
    if key == ord('s'):
        action = 's'

    # 1ステップ前の距離
    d = gps_mode.get_distance()
    regular_proc(camera)
    
    # スタック判定
    stuck_time = time.clock() - stuck_start
    print str(d-0.05) + " " + str(gps_mode.get_distance()) + " " + str(d+0.05) + " " + str(stuck_time)
    if not(d-0.05 <= gps_mode.get_distance() and gps_mode.get_distance() <= d+0.05):
        stuck_start = time.clock()
    # スタック
    if stuck_time >= 20:
        record = "stuck now"
        print record
        f.write(record + "\n")
        # スタック回避動作
        avoid_stuck()
        stuck_start = time.clock()

    # GPS Mode
    if mode == 0:
        control_byGPS()
            
    # Image Processing Mode
    else:
        control_byImg()
        if img_mode.get_redRate() > 50 and read_ultrasonic() < 50:
            return False


    return True

def regular_proc(camera):
    lon, lat = read_gps()
    if lon != 0 and lat != 0:
        gps_mode.set_robotGPS(lon, lat)
        gps_mode.set_robotDirection(read_compass())
        gps_mode.calc_distanceGPS()

    color_img = capture_image(camera, 1)
    binary_img = img_mode.extract_redColor(color_img, 160, 10, 70, 70)
    src = binary_img.copy()
    img_mode.find_centerPoint(src)
    draw_img(color_img, binary_img)

def control_byGPS():
    gps_mode.target_direction()
    action = gps_mode.get_action()
    gps_mode.robot_action()
    control_motor(action)
    record =  "[GPS mode] GPS: (" + str(gps_mode.get_robotX()) + ", " + str(gps_mode.get_robotY()) + ") Direction: (" + str(gps_mode.get_robotDirection()) + ", " + str(gps_mode.get_targetDirection()) + ") Distance: " + str(gps_mode.get_distance()*100) + "[cm] Action: " + action
    print record
    f.write(record + "\n")

def control_byImg():
    # 赤色を認識できないとき
    if img_mode.get_redRate() < 1.0:
        action = img_mode.get_action()
    else:
        img_mode.robot_action()
        action = img_mode.get_action()
    control_motor(action)
    record = "[IMG mode] GPS: (" + str(gps_mode.get_robotX()) + ", " + str(gps_mode.get_robotY()) + ") Point: (" + str(img_mode.get_targetX()) + ", " + str(img_mode.get_targetY()) + ") Distance: " + str(read_ultrasonic()) + "[cm] Action: " + action
    print record
    f.write(record + "\n")



def draw_img(original_img, binary_img):
    cv2.circle(original_img, (img_mode.get_targetX(), img_mode.get_targetY()), 10, (0, 255, 255), -1)
    cv2.putText(original_img, "RED: " + str('%3.2f' % img_mode.get_redRate()) + "[%]", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 127))
    cv2.imshow('original image', original_img)
    cv2.imshow('binary image', binary_img)

def avoid_stuck():
    motor.move_backward()
    time.sleep(5)
    motor.turn_right()
    time.sleep(5)

def control_motor(action):
    if action == 'f':
        motor.move_forward()
    elif action == 'b':
        motor.move_backward()
    elif action == 'r':
        motor.turn_right()
    elif action == 'l':
        motor.turn_left()
    elif action == 's':
        motor.stop()
    elif action == 'f':
        motion.finish()

def main():
    global stuck_start
    running_mode = 0   # 0->GPS mode, 1->IMG mode

    try:
        action = 's'
        mode = 0
        gps_mode.set_targetGPS(target_x, target_y)

        time.sleep(2)

        record =  "----------------------------------STANBY MODE----------------------------------"
        print record
        f.write(record + "\n")
        
        running_start = time.clock()
        
        
        with picamera.PiCamera() as camera:
        
            camera.resolution = (320, 240)
            time.sleep(2)
            while True:
                # standby mode
                if mode == 0:
                    if standby(camera) == False:
                        mode = 1
                        record = "----------------------------------FALLING MODE----------------------------------"
                        print record
                        f.write(record + "\n")
                # falling mode
                if mode == 1:
                    #falling()
                    motor.set_speed(50)
                    motor.move_forward()
                    time.sleep(10)
                    stuck_start = time.clock()
                    stop_start = time.clock()

                    mode = 2
                    record = "----------------------------------RUNNING MODE----------------------------------"
                    print record
                    f.write(record + "\n")
                    motor.set_speed(90)
                # running mode
                if mode == 2:
                    running_time = time.clock() - running_start
                    stop_time = time.clock() - stop_start
                    if running_time > 1:
                        if img_mode.get_redRate() > 3.0:
                            running_mode = 1
                            motor.set_speed(50)
                        if running(camera, running_mode) == False:
                            record = "----------------------------------!!! GOAL !!!----------------------------------"
                            print record
                            f.write(record + "\n")
                            break
                        else:
                            running_start = time.clock()
                    if stop_time > 60:
                        control_motor('s')
                        time.sleep(15)
                        stop_start = time.clock()
        

    except KeyboardInterrupt:
        motor.finish()
        GPIO.cleanup()
        f.close()

if __name__ == '__main__':
    main()
    
