# -*- coding:utf-8 -*-

import math
from collections import namedtuple

class GPSMode:

    def __init__(self):
        # x: 緯度, y: 経度, direction: 方位
        # robot: ロボット情報, target: ゴール情報
        BasicData = namedtuple('BasicData', 'x y direction')
        self.robot = BasicData(0, 0, 0)
        self.target = BasicData(0, 0, 0)
        # ロボット位置とゴール位置のそれぞの成分の差分
        self.sub_x = 0
        self.sub_y = 0
        # GPSによるゴールとの距離
        self.distance = 0
        # ロボットの動作
        self.action = 's'
            

    def calc_distanceGPS(self):
        # ロボット位置とゴール位置のそれぞの成分の差分
        self.sub_x = (self.target.x * 10000.0 - self.robot.x * 10000.0) * 11.11350
        self.sub_y = (self.target.y * 10000.0 - self.robot.y * 10000.0) * 9.11910
        self.distance = math.sqrt(self.sub_x * self.sub_x + self.sub_y * self.sub_y)

    def target_direction(self):
        if self.sub_x == 0.0:
            if self.sub_y >= 0.0:
                d = 0.0
            else:
                d = math.pi
        elif self.sub_y == 0.0:
            if self.sub_x >= 0.0:
                d = math.pi / 2.0
            else:
                d = 3.0 * math.pi / 2.0
        else:
            if self.sub_x > 0.0:
               d = math.atan2(self.sub_x, self.sub_y)
            else:
                d = math.atan2(self.sub_x, self.sub_y) + 2.0 * math.pi

        d = d / math.pi * 180.0

        if (348.75 <= d and d < 360.0) or (0.0 <= d and d < 11.25): self.target = self.target._replace(direction = 0)
        elif 11.25 <= d and d < 33.75: self.target = self.target._replace(direction = 1)
        elif 33.75 <= d and d < 56.25: self.target = self.target._replace(direction = 2)
        elif 56.25 <= d and d < 78.75: self.target = self.target._replace(direction = 3)
        elif 78.75 <= d and d < 101.25: self.target = self.target._replace(direction = 4)
        elif 101.25 <= d and d < 123.75: self.target = self.target._replace(direction = 5)
        elif 123.75 <= d and d < 146.25: self.target = self.target._replace(direction = 6)
        elif 146.25 <= d and d < 168.75: self.target = self.target._replace(direction = 7)
        elif 168.75 <= d and d < 191.25: self.target = self.target._replace(direction = 8)
        elif 191.25 <= d and d < 213.75: self.target = self.target._replace(direction = 9)
        elif 213.75 <= d and d < 236.25: self.target = self.target._replace(direction = 10)
        elif 236.25 <= d and d < 258.75: self.target = self.target._replace(direction = 11)
        elif 258.75 <= d and d < 281.25: self.target = self.target._replace(direction = 12)
        elif 281.25 <= d and d < 303.75: self.target = self.target._replace(direction = 13)
        elif 303.75 <= d and d < 326.25: self.target = self.target._replace(direction = 14)
        elif 326.25 <= d and d < 348.75: self.target = self.target._replace(direction = 15)

    def  robot_action(self):
        if self.robot.direction <= self.target.direction:
            n = self.target.direction - self.robot.direction
        else:
            n = self.target.direction - self.robot.direction + 16

        if n == 0:
            self.action = 'f'
        elif 0 < n and n <= 8:
            self.action = 'r'
        elif 8 < n and n <= 16:
            self.action = 'l'


    
        

    

    '''
    Set function
    '''
    def set_targetGPS(self, x, y):
        self.target = self.target._replace(x = x)
        self.target = self.target._replace(y = y)

    def set_robotGPS(self, x, y):
        self.robot = self.robot._replace(x = x)
        self.robot = self.robot._replace(y = y)

    def set_robotDirection(self, value):
        self.robot = self.robot._replace(direction = value)


    '''
    Get function
    '''
    def get_robotX(self):
        return self.robot.x

    def get_robotY(self):
        return self.robot.y

    def get_targetDirection(self):
        return self.target.direction

    def get_robotDirection(self):
        return self.robot.direction

    def get_distance(self):
        return self.distance

    def get_action(self):
        return self.action

    
