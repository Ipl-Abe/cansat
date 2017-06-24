import time
import picamera
import cv2

cv2.namedWindow('test')

def camera_capture():
        with picamera.PiCamera() as camera:    
            camera.resolution = (1024, 768)
            time.sleep(2)
            camera.capture('test.jpg')

            pic = cv2.imread('./test.jpg', 1)
            cv2.imshow('test', pic)
            cv2.waitKey(10)
   

def hello():
        print("hello")


capture_start = time.perf_counter()
hello_start = time.perf_counter()

while True:

    capture_time = time.perf_counter() - capture_start
    hello_time = time.perf_counter() - hello_start

    if hello_time > 10:
        hello()
        hello_start = time.perf_counter()

    if capture_time > 1:
        camera_capture()
        capture_start = time.perf_counter()

