import spidev
import math
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

spi = spidev.SpiDev() #Create an spi interface object

GPIO.setup(17, GPIO.OUT)
GPIO.output(17, GPIO.HIGH) #Latch dac if wanted
dac_ref = 3.3
dac_res = 8

def dac_set(voltage):
    spi.open(0, 1) #Open connection on (bus 0, cs/device 1)
    spi.max_speed_hz = 1350000
    spi.mode = 0b00

    spi.xfer2([0b00111000,0])

    spi.close()


for i in range(0, 7):
    dac_set(1.5)
    time.sleep(0.2)

GPIO.cleanup()
