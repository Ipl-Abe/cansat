import time
import picamera
import cv2


cv2.namedWindow('test')

def camera_capture(camera):
    camera.capture('test.jpg')
    frame = cv2.imread('./test.jpg', 1)
            
    return frame


with picamera.PiCamera() as camera:
    camera.resolution = (320, 240)
    time.sleep(2)

    start = time.clock()
    while True:
        t = time.clock() - start
        
        if t > 1:
            print t
            
            try:
                frame = camera_capture(camera)
                cv2.imshow('test', frame)
                cv2.waitKey(1)
                start = time.clock()


            except KeyboardInterrupt:
                break;

    cv2.destroyAllWindows()
