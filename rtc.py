from smbus2 import SMBus #Import i2c libary
import time
import datetime

rtc_address = 0x6f #Save the control code and cs bits 1101111

i2c = SMBus(1)

def rtc_init():
    #i2c.write_byte_data(rtc_address, 0x7, 0<<3) #Use the crystal osc (Default)
    if (not bool(i2c.read_byte_data(rtc_address, 0x0) & 1<<7)):
        i2c.write_byte_data(rtc_address, 0x0, 1<<7) #Start the oscilator
    #Optional Compensation
    i2c.write_byte_data(rtc_address, 0x7, 4) #Set to course trim mode (Bit 2)
    i2c.write_byte_data(rtc_address, 0x8, (1<<7) + 0) #Crystal Compensation amount

def rtc_set_time(hour, minute, second):
    print("Set the time")
    i2c.write_byte_data(rtc_address, 0x0, to_bcd(second) + (1<<7))
    i2c.write_byte_data(rtc_address, 0x1, to_bcd(minute))
    i2c.write_byte_data(rtc_address, 0x2, to_bcd(hour))

def rtc_hour():
    return to_deci(i2c.read_byte_data(rtc_address, 0x2)) & 63

def rtc_min():
    return to_deci(i2c.read_byte_data(rtc_address, 0x1))

def rtc_second():
    return to_deci(i2c.read_byte_data(rtc_address, 0x0)&127)

def rtc_makes(number): #'Make String' function
    if (number < 10):
        return str(0) + str(number)
    else:
        return str(number)

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

rtc_init() #Always call this on startup
dt = datetime.datetime.now()
#rtc_set_time(dt.hour, dt.minute, dt.second) #Set the time here
print("recall time")

i2c.write_byte_data(rtc_address, 0x8, (1<<7)) #behind

try:
    while (True):
        last_sec = rtc_second()
        while(last_sec == rtc_second()):
            time.sleep(0.001)
        print(rtc_time_string())

except KeyboardInterrupt:
    i2c.close()
