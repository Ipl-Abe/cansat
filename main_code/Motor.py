# -*- coding:utf-8 -*-

import RPi.GPIO as GPIO

class Motor:

    def __init__(self, r1, r2, l1, l2):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(r1, GPIO.OUT)
        GPIO.setup(r2, GPIO.OUT)
        GPIO.setup(l1, GPIO.OUT)
        GPIO.setup(l2, GPIO.OUT)

        self.r1 = GPIO.PWM(r1, 100)
        self.r2 = GPIO.PWM(r2, 100)
        self.l1 = GPIO.PWM(l1, 100)
        self.l2 = GPIO.PWM(l2, 100)
        self.r1.start(0)
        self.r2.start(0)
        self.l1.start(0)
        self.l2.start(0)
        self.speed = 0

    def move_forward(self):
        print self.speed
        self.r1.ChangeDutyCycle(0)
        self.r2.ChangeDutyCycle(self.speed)
        self.l1.ChangeDutyCycle(0)
        self.l2.ChangeDutyCycle(self.speed)

    def move_backward(self):
        self.r1.ChangeDutyCycle(self.speed)
        self.r2.ChangeDutyCycle(0)
        self.l1.ChangeDutyCycle(self.speed)
        self.l2.ChangeDutyCycle(0)

    def turn_right(self):
        self.r1.ChangeDutyCycle(0)
        self.r2.ChangeDutyCycle(self.speed/2)
        self.l1.ChangeDutyCycle(0)
        self.l2.ChangeDutyCycle(self.speed)

    def turn_left(self):
        self.r1.ChangeDutyCycle(0)
        self.r2.ChangeDutyCycle(self.speed)
        self.l1.ChangeDutyCycle(0)
        self.l2.ChangeDutyCycle(self.speed/2)

    def stop(self):
        self.r1.ChangeDutyCycle(0)
        self.r2.ChangeDutyCycle(0)
        self.l1.ChangeDutyCycle(0)
        self.l2.ChangeDutyCycle(0)

    def finish(self):
        self.r1.ChangeDutyCycle(0)
        self.r2.ChangeDutyCycle(0)
        self.l1.ChangeDutyCycle(0)
        self.l2.ChangeDutyCycle(0)
        self.r1.stop()
        self.r2.stop()
        self.l1.stop()
        self.l2.stop()

    def set_speed(self, speed):
        if 0 <= speed and speed <= 100:
            self.speed = speed
