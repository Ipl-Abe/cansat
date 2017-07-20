import spidev, time

spi = spidev.SpiDev()
spi.open(0, 0)

def analog_read(channel):
    r = spi.xfer2([1, (2 + channel) << 6, 0])
    adc_out = ((r[1]&31) << 6) + (r[2] >> 2)
    return adc_out

while True:
    reading = analog_read(0)
    volts = reading * 4.6 /1024
    inches = volts / 0.0098
    centis = inches * 2.54

    print(centis)

    time.sleep(1)
