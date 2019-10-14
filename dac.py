import spidev
import math
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

spi = spidev.SpiDev() #Create an spi interface object

dac_ref = 3.3
dac_res = 8

def dac_set(voltage):
    spi.open(0, 1) #Open connection on (bus 0, cs/device 1)
    spi.max_speed_hz = 1350000
    spi.mode = 0b00
    dac_data_bits = voltage_to_ddb(voltage)
    print(dac_data_bits)
    bits_6to10 = (dac_data_bits>>6)&(15) #Extract 4 MSBs
    bits_0to5 = dac_data_bits&(63) #Extract 6 LSBs
    print(str(bin(bits_6to10)) + str(bin(bits_0to5)))
    spi.xfer2([(0b0011<<4) + bits_6to10, bits_0to5<<2])
    spi.close()

def voltage_to_ddb(voltage):
    multiplier = 1023/3.3
    if (voltage > 3.3):
        return 1023
    elif (voltage < 0):
        return 0
    else:
        return int(multiplier*voltage)

try:
    while(1):
        for i in range(1, 4):
            dac_set(i)
            time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()
