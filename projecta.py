#!/usr/bin/python3
"""
Authors: Samantha Ball & Justin Wylie
Student Numbers: BLLSAM009, WYLJUS002
Prac: Project A
Date: 30/09/2019
"""

#TODO
#ADC - temp sensor, potentiometer(humidity), LDR (light)
#DAC - correctly calculated voltage
#Buttons - start/stop monitoring, dismiss alarm, change reading interval, reset system time
#Interrupts and debouncing
#RTC
#PWM output - alarm to notify issue, hard coded thresholds
#Alarm - can only go off every 3 minutes
#Connect to Blynk? - view live logging info (system time and ADC values), alarm notifications
#Print values to screen in correct format - RTC Time| Sys Timer| Humidity| Temp| Light| DAC Out| Alarm

#import Relevant Libraries
import threading
import time
import RPi.GPIO as GPIO
import spidev
import smbus
import math
import blynklib
import random
import datetime


###CONSTANTS###
RTCAddr = 0x6f;
SEC = 0x00;
MIN = 0x01;
HOUR = 0x02;
UPPER_BOUND = 2.65
LOWER_BOUND = 0.65
BLYNK_AUTH = 'UB-J4q8v4H8RkrOK8JEVL32B8jcUUgQi' #Auth Token
#READ_PRINT_MSG = "[READ_VIRTUAL_PIN_EVENT] Pin: V{}"



blynk = blynklib.Blynk(BLYNK_AUTH)

###GLOBALS###
#define pin numbers
#Buttons
start_btn = 12
freq_btn = 16
dismiss_btn = 20
reset_btn = 21

frequency = 1 #Default freq for monitoring is 1Hz
alarm_flag = False #Set when voltage boundry breached or recovers
alarm_dismissed = False #Once alarm is triggered, only stop alarming when it is user dismissed
running_flag = False

#SPI
I2C = None
spi = None
spi_ch = 0

#PWM LED
pwm = None
pwm_channel = 13

#ADC
analog_ref = 3.3
analog_res = 10
temp_coeff = 0.01 #10mV/C
v_degrees = 0.5  #500mv


#DAC
dac_ref = 3.3
dac_res = 8
dac_voltage = 0

#Measured Quantities
humidity = 0
light = 0
temp = 0
clock = 0
sys = 0

#Time keeping
time_zero = datetime.datetime(19, 10, 13, 11, 00, 00)

###THREADS###
def monitor_thread():
    global frequency
    global running_flag
    print("|RTC Time|Sys Timer|Humidity|Temp|Light|DAC out|Alarm")
    while (running_flag==True):
        monitor_adc();
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
            last_trigger = time.time() #start timer when alarm triggered
            while(not alarm_dismissed):
                pass
                #print("are we stuck here?")
            #last_trigger = time.time()
            pwm.ChangeDutyCycle(0)


###FUNCTIONS###
#test program to toggle LED
def main():
	#loop
        while(True):
        #print("Main looping")
        #time.sleep(5)
            #pass
            while True:
                blynk.run()
            """result = I2C.read_byte_data(RTCAddr, HOUR)
            print(result)
            result = I2C.read_byte_data(RTCAddr, MIN)
            print(result)
            result = I2C.read_byte_data(RTCAddr, SEC)
            print(result-0x80)"""
            #adc = spi.readbytes(8);
            #print(adc)




# register handler for virtual pin V11 reading
@blynk.handle_event('read V1')
def read_virtual_pin_handler(pin):
    global light
    global humidity
    global temp
    global clock
    global sys
    #print(READ_PRINT_MSG.format(pin))
    blynk.virtual_write(0, temp)
    blynk.virtual_write(pin, light)
    blynk.virtual_write(2, humidity)
    blynk.virtual_write(3, dac_voltage)

    if(alarm_flag==True):
         blynk.notify('Warning critical value') # send push notification
         blynk.virtual_write(4, 255) #turn on LED
    else:
        blynk.virtual_write(4, 0)
    header = "|RTC Time|Sys Timer|Humidity|Temp|Light|DAC out|Alarm"
    blynk.virtual_write(5, header)
    info = f"|{clock}|{sys}|{humidity:.1f} V|{temp_degrees:.0f} C|{light:.0f}|{dac_voltage: .2f}V|{alarm}|"
    print(info)
    blynk.virtual_write(5, info)
    #terminal
    #terminal.print("|RTC Time|Sys Timer|Humidity|Temp|Light|DAC out|Alarm")


#begin reading
def start_stop_method(channel):
        global running_flag
        if(running_flag==False):
                #start alarm thread
                alarmThread = threading.Thread(target=alarm_thread)
                alarmThread.daemon = True
                alarmThread.start()
	        #start monitor thread
                print("start/stop reading")
                monitorThread = threading.Thread(target=monitor_thread)
                monitorThread.daemon = True
                running_flag = True
                monitorThread.start()
        else:
                #monitorThread.wait??
                running_flag = False


#turn on PWM LED
def set_alarm(channel):
        print("alarm")
        alarmThread = threading.Thread(target=alarm_thread)
        alarmThread.daemon = True
        alarmThread.start()

#switch off PWM LED
def alarm_dismiss(channel):
        print("alarm dismissed")
        #pwm_led.stop
        global alarm_flag
        alarm_flag = False
        global alarm_dismissed
        alarm_dismissed = True


#change reading interval
def change_interval(channel):
        #print("change interval")
        #the possible frequencies are 1s, 2s and 5s.
        #The frequency must loop between those values per event occurrence
        print("Frequency toggled!")
        global frequency
        if (frequency == 1):
                frequency = 2
        elif (frequency == 2):
                frequency = 5
        else:
                frequency = 1

#reset system time
def reset_method(channel):
        global time_zero
        print("reset")
        time_zero = rtc_get_datetime()


#used to initialise all inputs and outputs
def GPIOInit():
        #choose Broadcom pin numbering
        GPIO.setmode(GPIO.BCM)

        #set up buttons with pull up resistors
        GPIO.setup(start_btn, GPIO.IN, pull_up_down=GPIO.PUD_UP) #start button 12
        GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_UP) #change read interval button
        GPIO.setup(20, GPIO.IN, pull_up_down=GPIO.PUD_UP) #alarm dismiss button
        GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_UP) #reset button

        #add edge detection to respond to push event
        GPIO.add_event_detect(12, GPIO.FALLING, callback=start_stop_method, bouncetime = 300)
        GPIO.add_event_detect(16, GPIO.FALLING, callback=change_interval, bouncetime = 300)
        GPIO.add_event_detect(20, GPIO.FALLING, callback=alarm_dismiss, bouncetime = 300)
        GPIO.add_event_detect(21, GPIO.FALLING, callback=reset_method, bouncetime = 300)

        #set up PWM LED
        global pwm
        GPIO.setup(pwm_channel, GPIO.OUT, initial=GPIO.LOW)
        pwm = GPIO.PWM(pwm_channel, 100) #1
        pwm.start(50)
        pwm.ChangeDutyCycle(0)


        #set up SPI pins
        #GPIO.setup(8, GPIO.OUT) #unnecessary?

#I2C needed for RTC
def init_I2C():
        global I2C
        global time_zero
        I2C = smbus.SMBus(1) # 1 indicates /dev/I2C-1
        address = 0x6f #whatever the device is for your I2C device
        if (not bool(I2C.read_byte_data(RTCAddr, 0x0) & 1<<7)):
            I2C.write_byte_data(RTCAddr, SEC, 0b10000000) #set ST bit
        rtc_set_time(time_zero)
    #Optional Compensation
    #I2C.write_byte_data(RTCAddr, 0x7, 4) #Set to course trim mode (Bit 2)
    #I2C.write_byte_data(RTCAddr, 0x8, (1<<7) + 0) #Crystal Compensation amount


def init_SPI():
    #DAC
    GPIO.setup(17, GPIO.OUT)
    GPIO.output(17, GPIO.HIGH) #Latch dac if wanted
    global spi
    spi = spidev.SpiDev()

#RTC FUNCTIONS
def rtc_set_time(dt): #Takes in a date time object
    print("Set the time")
    I2C.write_byte_data(RTCAddr, 0x0, to_bcd(dt.second) + (1<<7))
    I2C.write_byte_data(RTCAddr, 0x1, to_bcd(dt.minute))
    I2C.write_byte_data(RTCAddr, 0x2, to_bcd(dt.hour))
    I2C.write_byte_data(RTCAddr, 0x4, to_bcd(dt.day))
    I2C.write_byte_data(RTCAddr, 0x5, to_bcd(dt.month))
    I2C.write_byte_data(RTCAddr, 0x6, to_bcd(dt.year))


def rtc_year():
    return to_deci(I2C.read_byte_data(RTCAddr, 0x6)) & 63 #4 bit units, 4 bit tens (only use two)

def rtc_month():
    return to_deci(I2C.read_byte_data(RTCAddr, 0x5)) & 31 #4 bit units, 1 bit tens

def rtc_day():
    return to_deci(I2C.read_byte_data(RTCAddr, 0x4)) & 63 #4 bit units, 2 bit tens

def rtc_hour():
    return to_deci(I2C.read_byte_data(RTCAddr, 0x2)) & 63

def rtc_min():
    return to_deci(I2C.read_byte_data(RTCAddr, 0x1))

def rtc_second():
    return to_deci(I2C.read_byte_data(RTCAddr, 0x0)&127)

def rtc_makes(number): #'Make String' function
    if (number < 10):
        return str(0) + str(number)
    else:
        return str(number)

def rtc_get_datetime():
    return datetime.datetime(rtc_year(), rtc_month(), rtc_day(), rtc_hour(), rtc_min(), rtc_second())

def sys_time_string():
    global time_zero
    time_now = rtc_get_datetime()
    diff = (time_now - time_zero)
    return f"{diff}"#rtc_makes(diff.hour) + ":" + rtc_makes(diff.minute) + ":" + rtc_makes(diff.second)

def rtc_time_string():
    hour = rtc_hour()
    minute = rtc_min()
    second = rtc_second()
    return rtc_makes(hour) + ":" + rtc_makes(minute) + ":" + rtc_makes(second)

def to_bcd(num): #Assumes number less than 60
    units = num%10
    tens = (int(num/10))<<4
    return tens+units

def to_deci(num):
    units = num & 15
    tens = (num>>4)*10
    return tens+units


def adc_read(channel):
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


def monitor_adc():
    global alarm_flag
    global light
    global humidity
    global temp
    global dac_voltage

    temp = adc_read(0) #read temp sensor
    temp_degrees = (temp-v_degrees)/temp_coeff; #convert to Celsius
    temp = int(temp_degrees)
    time.sleep(0.1)
    humidity = adc_read(1) #read potentiometer representing humidity
    time.sleep(0.1)
    light = adc_read(2) #read LDR
    time.sleep(0.1)

    #dac_v = (light/1023)*humidity
    dac_v = dac_calc(light, humidity)
    dac_voltage = str(round(dac_v, 2))
    #print(dac_voltage)
    #dac_set(dac_v)
    if((dac_v<LOWER_BOUND)|(dac_v>UPPER_BOUND)):
        alarm_flag = True;
    if(alarm_flag==True):
        alarm = "*"
    else:
        alarm = ""
    clock = rtc_time_string()
    sys = sys_time_string()
    print_output(clock, sys, humidity, temp_degrees, light, dac_v, alarm)

def print_output(clock, sys, humidity, temp_degrees, light, dac_voltage, alarm):
    print(f"|{clock}|{sys}|{humidity:.1f} V|{temp_degrees:.0f} C|{light:.0f}|{dac_voltage: .2f}V|{alarm}|")

def dac_calc(light, humidity):
    dac_v = (light/1023)*humidity
    dac_set(dac_v)
    return dac_v


def dac_set(voltage):
    spi.open(0, 1) #Open connection on (bus 0, cs/device 1)
    spi.max_speed_hz = 1350000
    spi.mode = 0b00
    spi.xfer2([0b00111000,0])
    spi.close()

#Only run the functions if
if __name__ == "__main__":
    #Make sure the GPIO is stopped correctly
        try:
                GPIOInit()
                init_I2C();
                init_SPI();
                print("GPIO set up")
                while True:
                        main()
        except KeyboardInterrupt:
                print("Exiting gracefully")
                #Turn off your GPIOs here
                blynk.virtual_write(5, 'clr'); #clear terminal
                blynk.virtual_write(4, 0) #switch off alarm LED
                spi.close()
                GPIO.cleanup()
        except e:
                GPIO.cleanup()
                print("Some other error occurred")
                print(e.message)
