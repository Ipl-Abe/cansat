# -*- coding:utf-8 -*-

import time
import picamera
import cv2
import Motor
import GPSMode
import ImageProcMode

# target GPS data
target_x = 139.987270
target_y = 40.142732

'''
Pin Setting
'''
motor = Motor.Motor(26, 13, 19, 18, 23, 24)
gps_mode = GPSMode.GPSMode()
img_mode = ImageProcMode.ImageProcMode()

cv2.namedWindow('original image')
cv2.namedWindow('binary image')
        
def running(camera):
    key = cv2.waitKey(1)
    if key == ord('s'):
        action = 's'
        
    #control_byImg(camera)
    #print img_mode.get_targetX()

    control_byGPS()
    print str(gps_mode.get_robotX()) + ", " + str(gps_mode.get_robotY())

def control_byGPS():
    gps_mode.read_robotGPS()
    gps_mode.calc_distanceGPS()

def control_byImg(camera):
    
    color_img = img_mode.capture_image(camera)
    binary_img = img_mode.extract_redColor(color_img, 160, 10, 70, 70)
    src = binary_img.copy()
    img_mode.find_centerPoint(src)
    draw_img(color_img, binary_img)

def draw_img(original_img, binary_img):
    cv2.circle(original_img, (img_mode.get_targetX(), img_mode.get_targetY()), 10, (0, 255, 255), -1)
    cv2.putText(original_img, "RED: " + str('%3.2f' % img_mode.get_redRate()) + "[%]", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 127))
    cv2.imshow('original image', original_img)
    cv2.imshow('binary image', binary_img)

def control_motor():
    if self.action == 'f':
        self.motor.move_forward()
    elif self.action == 'b':
        self.motor.move_backward()
    elif self.action == 'r':
        self.motor.turn_right()
    elif self.action == 'l':
        self.motor.turn_left()
    elif self.action == 's':
        self.motor.stop()
    elif self.action == 'f':
        self.motion.finish()

def main():

    action = 's'
    gps_mode.set_targetGPS(target_x, target_y)

    time.sleep(1)

    print "----------------------------------STANBY MODE----------------------------------"

    running_start = time.clock()

    with picamera.PiCamera() as camera:
        camera.resolution = (320, 240)
        time.sleep(2)
        while True:
            running_time = time.clock() - running_start
            if running_time > 1:
                running(camera)
                running_start = time.clock()

if __name__ == '__main__':
    main()
    
