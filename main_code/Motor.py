# -*- coding:utf-8 -*-

import RPi.GPIO as GPIO

class Motor:

    def __init__(self, pwmR, r1, r2, pwmL, l1, l2):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pwmR, GPIO.OUT)
        GPIO.setup(r1, GPIO.OUT)
        GPIO.setup(r2, GPIO.OUT)
        GPIO.setup(pwmL, GPIO.OUT)
        GPIO.setup(l1, GPIO.OUT)
        GPIO.setup(l2, GPIO.OUT)

        self.r1 = r1
        self.r2 = r2
        self.l1 = l1
        self.l2 = l2
        self.pwmR = GPIO.PWM(pwmR, 50)
        self.pwmR.start(0)
        self.pwmL = GPIO.PWM(pwmL, 50)
        self.pwmL.start(0)
        self.speed = 0

    def move_forward(self):
        self.pwmR.ChangeDutyCycle(self.speed)
        self.pwmL.ChangeDutyCycle(self.speed)
        GPIO.output(self.r1, 1)
        GPIO.output(self.r2, 0)
        GPIO.output(self.l1, 1)
        GPIO.output(self.l2, 0)

    def move_backward(self):
        self.pwmR.ChangeDutyCycle(self.speed)
        self.pwmL.ChangeDutyCycle(self.speed)
        GPIO.output(self.r1, 0)
        GPIO.output(self.r2, 1)
        GPIO.output(self.l1, 0)
        GPIO.output(self.l2, 1)

    def turn_right(self):
        self.pwmR.ChangeDutyCycle(self.speed-10)
        self.pwmL.ChangeDutyCycle(self.speed)
        GPIO.output(self.r1, 1)
        GPIO.output(self.r2, 0)
        GPIO.output(self.l1, 1)
        GPIO.output(self.l2, 0)

    def turn_left(self):
        self.pwmR.ChangeDutyCycle(self.speed)
        self.pwmL.ChangeDutyCycle(self.speed-10)
        GPIO.output(self.r1, 1)
        GPIO.output(self.r2, 0)
        GPIO.output(self.l1, 1)
        GPIO.output(self.l2, 0)

    def stop(self):
        GPIO.output(self.r1, 0)
        GPIO.output(self.r2, 0)
        GPIO.output(self.l1, 0)
        GPIO.output(self.l2, 0)

    def finish(self):
        self.pwmR.stop()
        self.pwmL.stop()

    def set_speed(self, speed):
        if 0 <= speed and speed <= 100:
            self.speed = speed
            self.pwmR.ChangeDutyCycle(self.speed)
            self.pwmL.ChangeDutyCycle(self.speed)
