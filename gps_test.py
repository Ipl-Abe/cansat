import time
import serial
from pynmea import nmea
from gps3 import *

ser = serial.Serial('/../dev/ttyS0', 9600)
gpgga = nmea.GPGGA()

def test():
        session = gps()
        session.stream(WATCH_ENABLE|WATCH_NEWSTYLE)

        while True:
                report = session.next()
                if report.keys()[0] == 'exp' :
                        lat = float(report['lat'])
                        lon = float(report['lon'])
                        print("Latitude: " + str(lat))
                        print("Longitude: " + str(lon))
                        time.sleep(0.5)


def get_gps():

        gpsLat = -1
        gpsLong = -1
        
        data = ser.readline()
        data = data.decode('utf-8')

        if "$GPGGA" in data :
                try:
                        gpgga.parse(data)
                        gpggaLat = gpgga.latitude
                        gpggaLong = gpgga.longitude


                        if gpggaLat is not ' ':
                
                                gpsLat = float(gpggaLat[0:2]) + float(gpggaLat[2:]) / 60
                                gpsLong = float(gpggaLong[0:3]) + float(gpggaLong[3:]) / 60

                except ValueError:
                        gpsLat = -1
                        gpsLong = -1
                        
        return gpsLat, gpsLong


#test()

x = -1
y = -1

while True:

        time.sleep(0.5)

        lat, long= get_gps()

        if lat is not -1:
                x = lat
        if long is not -1:
                y = long

        print("Latitude: " + str(x))
        print("Longitude: " + str(y))


