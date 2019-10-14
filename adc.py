import spidev
import math
import RPi.GPIO as GPIO
import time

spi = spidev.SpiDev() #Create an spi interface object

analog_ref = 3.3
analog_res = 10

def adc_read_channel(channel):
    spi.open(0, 0) #Open connection on (bus 0, cs/device 0)
    spi.max_speed_hz = 1350000
    spi.mode = 0b00
    bytes = spi.xfer2([1,(8+channel)<<4,0]) #Read from ADC
    spi.close()
    value = ((bytes[1]&3)<<8)+bytes[2]
    if(channel!=2):
        voltage = (analog_ref/(math.pow(2,analog_res)-1))*value
    else:
        voltage = value
    return voltage

def adc_read(pin):
    spi.open(0, 0) #Open connection on (bus 0, cs/device 0)
    spi.max_speed_hz = 1350000
    spi.mode = 0b00

    #print(bin((8+(pin&7))))
    #bytes = spi.xfer2([(24+(pin&7))<<3,0,0])
    #bytes = spi.xfer2([1,(8+1)<<4,0])
    bytes = spi.xfer2([1,(8+1)<<4,0]) #Second channel of the ADC

    spi.close()

    value = ((bytes[1]&3)<<8)+bytes[2]
    print(bin(bytes[0]))
    print(bin(bytes[1]))
    print(bin(bytes[2]))
    voltage = (analog_ref/(math.pow(2,analog_res)-1))*value
    return voltage

while(1):
    for i in range(0, 7):
        print(str(adc_read_channel(i)) + "v")
    print("++++++++++++++++++++++++++++++++++++++++")
    time.sleep(1)
