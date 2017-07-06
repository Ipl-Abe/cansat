# -*- coding:utf-8 -*-

import cv2
import picamera
import spidev
import numpy as np
import math
from collections import namedtuple

class ImageProcMode:

    def __init__(self):
        # row: 画像の列数（幅）, column: 画像の行数（高さ）
        self.row = 0
        self.column = 0
        #
        # 画像中のゴールの中心座標
        Point2d = namedtuple('Point2d', 'x y')
        self.target = Point2d(0, 0)
        # 距離センサによるゴールとの距離
        self.distance = 0
        # 画像中の赤色の割合
        self.red_rate = 0
         # ロボットの動作
        self.action = 's'

        cv2.namedWindow('original image')
        cv2.namedWindow('binary image')

    def capture_image(self, camera):
        camera.capture('original_image.jpg')
        img = cv2.imread('./original_image.jpg', 1)

        return img
    
    # color_extract(): カラー画像から赤色を白，その他の色を黒とした２値画像を生成する
    # src     :  元画像
    # h_th_low: Hのしきい値の最小値
    # h_th_up : Hのしきい値の最大値
    # s_th    : Sのしきい値の最小値(最大値は255)
    # v_th    : Vのしきい値の最小値(最大値は255)

    def extract_redColor(self, src, h_th_low, h_th_up, s_th, v_th):
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

    def find_centerPoint(self, src):
        max_id = -1
        max_area = 0
        c = np.array([0, 0])
        self.row, self.column = src.shape

        img, contours, hierarchy = cv2.findContours(src, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        for i in range(0, len(contours)):
            area = cv2.contourArea(contours[i])
            if area > max_area:
                max_id = i
                max_area = area

        self.red_rate = max_area / (self.row * self.column) * 100
            
        if max_id != -1:
            m = cv2.moments(contours[max_id])
            if m['m10'] == 0 or m['m00']  == 0:
                self.target = self.target._replace(x = 0)
            else:
                self.target = self.target._replace(x = int(m['m10'] / m['m00']))
            if m['m01'] == 0 or m['m00'] == 0:
                self.target = self.target._replace(y = 0)
            else:
                self.target = self.target._replace(y = int(m['m01'] / m['m00']))

    def read_analogData(self, channel):
        r = spi.xfer2([1, (8 + channel) << 4, 0])
        adc_out = ((r[1]&3) << 8) + r[2]
        return adc_out

    def read_ultrasonic(self):
        analog = read_analogData(0)
        volts = analog * 5.0 / 1024
        inches = volts / 0.0098
        self.distance = inches * 2.54      

    def robot_action(self):
        if 0 <= self.target.x and self.target.x < self.row/2:
            self.action = 'l'
        elif self.target.x == self.row/2:
            self.action = 'f'
        elif self.row/2 < self.target.x and self.target.x <= self.row:
            self.action = 'r'

    '''
    Set function
    '''

    '''
    Get function
    '''
    def get_targetX(self):
        return self.target.x

    def get_targetY(self):
        return self.target.y
    
    def get_distance(self):
        return self.distance

    def get_redRate(self):
        return self.red_rate

    def get_action(self):
        return self.action
