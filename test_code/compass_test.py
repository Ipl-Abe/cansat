import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

channels = [6, 13, 19, 26]
sensor = [0, 0, 0, 0]
GPIO.setup(channels, GPIO.IN)

for i in range(4):
    sensor[i] = GPIO.input(channels[i])

c = 0
if sensor[0] == 1: c = c + 1
if sensor[1] == 1: c = c + 2
if sensor[2] == 1: c = c + 4
if sensor[3] == 1: c = c + 8

if c == 0:
    print "N"
elif c == 1:
    print "NNE"
elif c == 2:
    print "NE"
elif c == 3:
    print "ENE"
elif c == 4:
    print "E"
elif c == 5:
    print "ESE"
elif c == 6:
    print "SE"
elif c == 7:
    print "SSE"
elif c == 8:
    print "S"
elif c == 9:
    print "SSW"
elif c == 10:
    print "SW"
elif c == 11:
    print "WSW"
elif c == 12:
    print "W"
elif c == 13:
    print "WNW"
elif c == 14:
    print "NW"
elif c == 15:
    print "N"

