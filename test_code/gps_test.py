import time
import csv
from gps import *


f = open('gps_test.csv', 'w')
writer = csv.writer(f, lineterminator='\n')

print "start"
session = gps()

session.stream(WATCH_ENABLE|WATCH_NEWSTYLE)

while True:

        #time.sleep(0.5)

        #print "test"
        try:
                report = session.next()
                if report.keys()[0] == 'epx' :
                        lat = float(report['lat'])
                        lon = float(report['lon'])
                        gpsdata = []
                        gpsdata.append(lat)
                        gpsdata.append(lon)
                        writer.writerow(gpsdata)
                        print "Latitude: " + str(lat)
                        print "Longitude: " + str(lon)
                        time.sleep(0.5)
        except KeyboardInterrupt:
                f.close()
                print "close"
                break

