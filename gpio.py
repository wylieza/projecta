import RPi.GPIO as GPIO
import time

led = 13
btn1 = 21
btn2 = 20
btn3 = 16
btn4 = 12

GPIO.setmode(GPIO.BCM)

GPIO.setup(led, GPIO.OUT)
GPIO.setup(btn1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(btn2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(btn3, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(btn4, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def debounce():
    time.sleep(0.001)

def falling1(channel):
    debounce()
    GPIO.output(led, GPIO.HIGH)

def falling2(channel):
    debounce()
    GPIO.output(led, GPIO.LOW)

GPIO.add_event_detect(btn1, GPIO.FALLING, callback=falling1)
GPIO.add_event_detect(btn2, GPIO.FALLING, callback=falling2)

try:
    while(True):
        print("LOOPING!!!!")
except KeyboardInterrupt:
    GPIO.cleanup

'''
try:
    while(True):
        if (GPIO.input(btn1) == GPIO.LOW):
            GPIO.output(led, GPIO.HIGH)
        else:
            GPIO.output(led, GPIO.LOW)
except KeyboardInterrupt:
    GPIO.cleanup
'''
