########################
#IMPORTS
########################
import RPi.GPIO as GPIO
import time
import threading


########################
#GLOBALS
########################
led = 13
btn1 = 21
btn2 = 20
btn3 = 16
btn4 = 12
frequency = 1 #Default freq for monitoring is 1Hz
alarm_flag = False #Set when voltage boundry breached or recovers
alarm_dismissed = False #Once alarm is triggered, only stop alarming when it is user dismissed
dac_voltage = 0

########################
#SETUP
########################
#Board mode
GPIO.setmode(GPIO.BCM)
#Outputs
GPIO.setup(led, GPIO.OUT)
pwm = GPIO.PWM(led, 100) #Setup 100Hz led PWM
pwm.start(50)
#pwm.ChangeDutyCycle(50)
#pwm.stop()
#Inputs
GPIO.setup(btn1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(btn2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(btn3, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(btn4, GPIO.IN, pull_up_down=GPIO.PUD_UP)


########################
#FUNCTIONS
########################
#Interrupt Routines
def alarm_dismiss(channel):
    global alarm_dismissed
    alarm_dismissed = True

def reset(channel): #resets the system timer and cleans the console
    global alarm_flag #TESTING
    alarm_flag = not alarm_flag #TESTING

def frequency_toggle(channel): #Toggle between the 3 frequencies
    print("Frequency toggled!")
    global frequency
    if (frequency == 1):
        frequency = 2
    elif (frequency == 2):
        frequency = 5
    else:
        frequency = 1

def stop(channel): #Toggle the monitoring of sensors
    #TODO -> Toggle monitoring
    pass

#I2C Interfacing
#RTC


#SPI Interfacing
#ADC

#DAC

########################
#THREADS
########################
def monitor_thread():
    global frequency
    global alarm_flag
    while (True):
        print("Thread boi updating")



        time.sleep(1/frequency)

def alarm_thread():
    global pwm
    global alarm_flag
    global alarm_dismissed
    last_trigger = 0
    alarm_flag = False
    while(True):
        if (alarm_flag and (time.time()-last_trigger > 10)):
            alarm_dismissed = False
            pwm.ChangeDutyCycle(100)
            while(not alarm_dismissed):
                pass
            last_trigger = time.time()
            pwm.ChangeDutyCycle(0)

########################
#Setup Interrupts
########################
GPIO.add_event_detect(btn1, GPIO.FALLING, callback=alarm_dismiss, bouncetime=200)
GPIO.add_event_detect(btn2, GPIO.FALLING, callback=reset, bouncetime=200)
GPIO.add_event_detect(btn3, GPIO.FALLING, callback=frequency_toggle, bouncetime=200)
GPIO.add_event_detect(btn4, GPIO.FALLING, callback=stop, bouncetime=200)

########################
#Startup Threads
########################
monitorThread = threading.Thread(target=monitor_thread)
alarmThread = threading.Thread(target=alarm_thread)
monitorThread.daemon = True
alarmThread.daemon = True
monitorThread.start()
alarmThread.start()

try:
    #Program Main
    while(True):
        print("Main looping")
        time.sleep(5)
except KeyboardInterrupt:
    print("Interrupted!")
    GPIO.cleanup()
