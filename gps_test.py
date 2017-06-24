import time
import serial
from pynmea import nmea

def get_gps():

        gpsLat = -1
        gpsLong = -1
        
        data = ser.readline()
        data = data.decode('utf-8')

        if "$GPGGA" in data :
                gpgga.parse(data)
                gpggaLat = gpgga.latitude
                gpggaLong = gpgga.longitude


                if gpggaLat is not '' :
                
                        gpsLat = float(gpggaLat[0:2]) + float(gpggaLat[2:]) / 60
                        gpsLong = float(gpggaLong[0:3]) + float(gpggaLong[3:]) / 60

        return gpsLat, gpsLong



ser = serial.Serial('/../dev/ttyS0', 9600)
gpgga = nmea.GPGGA()




while True:

        time.sleep(1)
        x, y = get_gps()
        print("Lat : " + str(x))
        print("Long : " + str(y))
