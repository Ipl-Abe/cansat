import time
import picamera
import cv2
import numpy as np

cv2.namedWindow('test')
cv2.namedWindow('binary image')

def camera_capture():
        with picamera.PiCamera() as camera:    
            camera.resolution = (512, 384)
            time.sleep(2)
            camera.capture('test.jpg')
            frame = cv2.imread('./test.jpg', 1)
            
        return frame

# color_extract(): カラー画像から赤色を白，その他の色を黒とした２値画像を生成する      
# src     :  元画像
# h_th_low: Hのしきい値の最小値
# h_th_up : Hのしきい値の最大値
# s_th    : Sのしきい値の最小値(最大値は255)
# v_th    : Vのしきい値の最小値(最大値は255)
def extract_redColor(src, h_th_low, h_th_up, s_th, v_th):
        # BGR➜HSV
        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        # HSVの各チャンネルを別々の画像に分ける
        h, s, v = cv2.split(hsv)

        # H(色相)に関する２値化
        # しきい値が両端にある場合（主に赤のとき）
        if h_th_low > h_th_up:  
                # cv2.THRESH_BINARY: しきい値より大きい場合は最大値，それ以外は0とする
                # cv2.THRESH_BINARY_INV: しきい値より大きい場合は0，それ以外は最大値とする
                ret, h_dst_1 = cv2.threshold(h, h_th_low, 255, cv2.THRESH_BINARY)
                ret, h_dst_2 = cv2.threshold(h, h_th_up, 255, cv2.THRESH_BINARY_INV)

                h_dst = cv2.bitwise_or(h_dst_1, h_dst_2)
        # しきい値が固まってあるとき
        else:
                # cv2.THRESH_TOZERO: しきい値より大きい場合はそのまま，それ以外は0とする
                # cv2.THRESH_TOZERO_INV: しきい値より大きい場合は0，それ以外はそのままとする
                ret, dst = cv2.threshold(h, h_th_low, 255, cv2.THRESH_TOZERO)
                ret, dst = cv2.threshold(h, h_th_up, 255, cv2.THRESH_TOZERO_INV)

                ret, h_dst = cv2.threshold(dst, 0, 255, cv2.THRESH_BINARY)

        # S(明度)に関する２値化
        ret, s_dst = cv2.threshold(s, s_th, 255, cv2.THRESH_BINARY)
        # V(彩度)に関する２値化
        ret, v_dst = cv2.threshold(v, v_th, 255, cv2.THRESH_BINARY)

        # HSVそれぞれのしきい値を満たす画素の抽出
        dst = cv2.bitwise_and(h_dst, s_dst)
        dst = cv2.bitwise_and(dst, v_dst)

        return dst

def find_centerPoint(src):
        max_id = -1
        max_area = 0
        c = np.array([0, 0])
        row, column = src.shape

        img, contours, hierarchy = cv2.findContours(src, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        #cv2.drawContours(img, contours, -1, (255, 255, 255), 3)

        for i in range(0, len(contours)):
                area = cv2.contourArea(contours[i])
                if area > max_area:
                        max_id = i
                        max_area = area

        red_rate = max_area / (row * column) * 100
        print(red_rate)

        
        if max_id != -1:
                m = cv2.moments(contours[max_id])
                if m['m10'] == 0 or m['m00']  == 0:
                        c[0] = 0
                else:
                        c[0] = int(m['m10'] / m['m00'])
                if m['m01'] == 0 or m['m00'] == 0:
                        c[1] = 0
                else:
                        c[1] = int(m['m01'] / m['m00'])

        #cv2.circle(src, (c[0], c[1]), 1000, (0, 0, 255), -1)
        #print(c)

        return red_rate, c

def draw_img(img, binary_img, p):
        cv2.circle(img, (p[0], p[1]), 10, (0, 255, 255), -1)
        cv2.imshow('test', img)
        cv2.imshow('binary image', binary_img)


def main():
        capture_start = time.perf_counter()
        hello_start = time.perf_counter()

        while True:
                capture_time = time.perf_counter() - capture_start
                hello_time = time.perf_counter() - hello_start

                if capture_time > 1:
                        img = camera_capture()
                        
                        binary_img = extract_redColor(img, 160, 10, 70, 70)
                    
                        src = binary_img.copy()
                        red_rate, p = find_centerPoint(src)
                        draw_img(img, binary_img, p)
                
                        capture_start = time.perf_counter()

                key = cv2.waitKey(1)
                if key == ord('q'):
                        break

        cv2.destroyAllWindows()



if __name__ == '__main__':
        main()
