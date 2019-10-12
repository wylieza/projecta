#!/usr/bin/python3
"""
Name: Samantha Ball
Student Number: BLLSAM009
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



###CONSTANTS###
RTCAddr = 0x6f;
SEC = 0x00; 
MIN = 0x01;
HOUR = 0x02;

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
            while(not alarm_dismissed):
                pass
            last_trigger = time.time()
            pwm.ChangeDutyCycle(0)


###FUNCTIONS###
#test program to toggle LED
def main():
	#loop
        while(True):
        #print("Main looping")
        #time.sleep(5)
            pass
            """result = I2C.read_byte_data(RTCAddr, HOUR)
            print(result)
            result = I2C.read_byte_data(RTCAddr, MIN)
            print(result)
            result = I2C.read_byte_data(RTCAddr, SEC)
            print(result-0x80)"""
            #adc = spi.readbytes(8);
            #print(adc)
    
#begin reading
def start_stop_method(channel):
        global running_flag
        if(running_flag==False):
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
        #pwm_led.ChangeFrequency(freq)
        #pwm_led.ChangeDutyCycle(dc)
        #pwm_led.start(dc)
	#Note that PWM will also stop if the instance variable 'pwm_led' goes out of scope. (NB!!) - make global?

#switch off PWM LED
def alarm_dismiss(channel):
        print("alarm dismissed")
        #pwm_led.stop
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
        print("reset")
        #reset system time
        #write to RTC??
    
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
        GPIO.setup(pwm_channel, GPIO.OUT, initial=GPIO.LOW)
        pwm_led = GPIO.PWM(pwm_channel, 1)


        #set up SPI pins
        #GPIO.setup(8, GPIO.OUT) #unnecessary?

#I2C needed for RTC
def init_I2C():
        global I2C
        I2C = smbus.SMBus(1) # 1 indicates /dev/i2c-1
        address = 0x6f #whatever the device is for your i2c device
        I2C.write_byte_data(RTCAddr, SEC, 0b10000000) #set ST bit 
        result = I2C.read_byte_data(RTCAddr, HOUR)
        print(result)


def init_SPI():
    #DAC
    GPIO.setup(17, GPIO.OUT)
    GPIO.output(17, GPIO.HIGH) #Latch dac if wanted   
    global spi
    spi = spidev.SpiDev() 

"""#SPI needed for ADC and DAC
def SpiInit():
    global spi
    #Bus is 0 or 1, depending on which SPI bus you’ve connected to
    bus = 0 #0
    #Device is the chip select pin. Set to 0 or 1, depending on the connections
    device = 1
    spi = spidev.SpiDev() #Enable SPI
    spi.open(bus, device) #Open connection to a specific bus and device (CS pin)
    # Set settings (SPI speed and mode)
    spi.max_speed_hz = 500000
    spi.mode = 0 
    #to_send = [0x01, 0x02, 0x03] #define what to send
    #spi.xfer(to_send)
    spi.writebytes(0b
    adc = spi.readbytes(8);
    print(adc)
    # Close the SPI connection
    #spi.close()"""

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
    temp = adc_read(0) #read temp sensor
    #print(temp)
    temp_degrees = (temp-v_degrees)/temp_coeff;
    time.sleep(0.1)
    #print(str(temp_degrees) + "v")
    humidity = adc_read(1) #read potentiometer representing humidity
    #print(str(humidity) + "v")
    time.sleep(0.1)
    light = adc_read(2)
    #print(str(light) + "v")
    time.sleep(0.1)

    dac_voltage = (light/1023)*humidity
    #print(dac_voltage)
    dac_set(dac_voltage)
    alarm = "*"
    clock = "10:17:15"
    sys = "00:00:00"
    print_output(clock, sys, humidity, temp_degrees, light, dac_voltage, alarm)

def print_output(clock, sys, humidity, temp_degrees, light, dac_voltage, alarm):
    print(f"|{clock}|{sys}|{humidity:.1f} V|{temp_degrees:.0f} C|{light:.0f}|{dac_voltage: .2f}V|{alarm}|") 

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
                spi.close()
                GPIO.cleanup()
        except e:
                GPIO.cleanup()
                print("Some other error occurred")
                print(e.message)
