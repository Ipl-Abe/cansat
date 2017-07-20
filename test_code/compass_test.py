import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

channels = [22, 27, 17, 4]
sensor = [0, 0, 0, 0]
GPIO.setup(channels, GPIO.IN)

while True:

    for i in range(4):
        sensor[i] = GPIO.input(channels[i])

    c = 0
    if sensor[0] == 1: c = c + 1
    if sensor[1] == 1: c = c + 2
    if sensor[2] == 1: c = c + 4
    if sensor[3] == 1: c = c + 8

    s =  str(sensor[3]) + str(sensor[2]) + str(sensor[1]) + str(sensor[0])

    if c == 0:
        print s + ": 8 S"
    elif c == 1:
        print s +": 11 WSW"
    elif c == 2:
        print s +": 3 ENE"
    elif c == 3:
        print s +": 0 N"
    elif c == 4:
        print s +": 7 SSE"
    elif c == 5:
        print s +": 12 W"
    elif c == 6:
        print s +": 4 E"
    elif c == 7:
        print s +": 15 NNW"
    elif c == 8:
        print s +": 9 SSW"
    elif c == 9:
        print s +": 10 SW"
    elif c == 10:
        print s +": 2 NE"
    elif c == 11:
        print s +": 1 NNE"
    elif c == 12:
        print s +": 6 SE"
    elif c == 13:
        print s +": 13 WNW"
    elif c == 14:
        print s +": 5 ESE"
    elif c == 15:
        print s +": 14 NW"

    time.sleep(1)
