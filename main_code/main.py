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

# target GPS data
target_x = 139.939475
target_y = 37.522963333

'''
Pin Setting
'''
motor = Motor.Motor(23, 24, 7, 25)
gps_mode = GPSMode.GPSMode()
img_mode = ImageProcMode.ImageProcMode()

'''
Compass sensor Setting
'''
GPIO.setmode(GPIO.BCM)
channels = [22, 27, 17, 4]
GPIO.setup(channels, GPIO.IN)
compass = [0, 0, 0, 0]

'''
GPS Setting
'''
session = gps()
session.stream(WATCH_ENABLE|WATCH_NEWSTYLE)

'''
Ultrasonic sensor Setting
'''
spi = spidev.SpiDev()
spi.open(0, 0)


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
        r = spi.xfer2([1, (8 + channel) << 4, 0])
        adc_out = ((r[1]&3) << 8) + r[2]
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
    print np.average(img)
    if np.average(img) > 30:
        return False
    else:
        return True
    
def running(camera):
    key = cv2.waitKey(1)
    if key == ord('s'):
        action = 's'

    distance = 10000
        
    control_byImg(camera)
    # Image Processing Mode
    if img_mode.get_redRate() > 5:
        distance = read_ultrasonic()
        img_mode.robot_action()
        action = img_mode.get_action()
        control_motor(action)
        print "Point: (" + str(img_mode.get_targetX()) + ", " + str(img_mode.get_targetY()) + ") Distance: " + str(distance) + " Action: " + action
        if distance < 50:
            return False

    # GPS Mode
    else:
        control_byGPS()
        action = gps_mode.get_action()
        control_motor(action)
        print "GPS: (" + str(gps_mode.get_robotX()) + ", " + str(gps_mode.get_robotY()) + ") Direction: (" + str(gps_mode.get_robotDirection()) + ", " + str(gps_mode.get_targetDirection()) + ") Distance: " + str(gps_mode.get_distance()) + "Action: " + action
        if gps_mode.get_distance() < 5:
            return False

    return True

def control_byGPS():
    lon, lat = read_gps()
    if lon != 0 and lat != 0:
        gps_mode.set_robotGPS(lon, lat)
    gps_mode.set_robotDirection(read_compass())
    gps_mode.target_direction()
    gps_mode.calc_distanceGPS()
    gps_mode.robot_action()

def control_byImg(camera):
    color_img = capture_image(camera, 1)
    binary_img = img_mode.extract_redColor(color_img, 160, 10, 70, 70)
    src = binary_img.copy()
    img_mode.find_centerPoint(src)
    draw_img(color_img, binary_img)

def draw_img(original_img, binary_img):
    cv2.circle(original_img, (img_mode.get_targetX(), img_mode.get_targetY()), 10, (0, 255, 255), -1)
    cv2.putText(original_img, "RED: " + str('%3.2f' % img_mode.get_redRate()) + "[%]", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 127))
    cv2.imshow('original image', original_img)
    cv2.imshow('binary image', binary_img)

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

    action = 's'
    mode = 0
    gps_mode.set_targetGPS(target_x, target_y)

    time.sleep(1)

    print "----------------------------------STANBY MODE----------------------------------"

    running_start = time.clock()

    with picamera.PiCamera() as camera:
        camera.resolution = (320, 240)
        time.sleep(2)
        while True:
            # standby mode
            if mode == 0:
                if standby(camera) == False:
                    mode = 1
                    print "----------------------------------FALLING MODE----------------------------------"
            # falling mode
            if mode == 1:
                mode = 2
                print "----------------------------------RUNNING MODE----------------------------------"
            # running mode
            if mode == 2:
                motor.set_speed(100)
                running_time = time.clock() - running_start
                if running_time > 1:
                    if running(camera) == False:
                        print "----------------------------------!!! GOAL !!!----------------------------------"
                        break
                    running_start = time.clock()

if __name__ == '__main__':
    main()
    
