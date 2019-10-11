from smbus2 import SMBus #Import i2c libary
import time

rtc_address = 0x6f #Save the control code and cs bits 1101111

i2c = SMBus(1)

def rtc_init():
    #i2c.write_byte_data(rtc_address, 0x7, 0<<3) #Use the crystal osc (Default)
    i2c.write_byte_data(rtc_address, 0x0, 1<<7) #Start the oscilator
    i2c.write_byte_data(rtc_address, 0x8, (1<<7) + 52)
    i2c.write_byte_data(rtc_address, 0x7, 4)

def rtc_set_time(hour, minute, second):
    print("Set the time")
    i2c.write_byte_data(rtc_address, 0x0, to_bcd(second) + (1<<7))
    i2c.write_byte_data(rtc_address, 0x1, to_bcd(minute))
    i2c.write_byte_data(rtc_address, 0x2, to_bcd(hour))

def rtc_read_time():
    hour = to_deci(i2c.read_byte_data(rtc_address, 0x2))
    minute = to_deci(i2c.read_byte_data(rtc_address, 0x1))
    second = to_deci(i2c.read_byte_data(rtc_address, 0x0)&127)
    print("Time:" + str(hour) + ":" + str(minute) + ":" + str(second))

def to_bcd(num): #Assumes number less than 60
    units = num%10
    tens = (int(num/10))<<4
    return tens+units

def to_deci(num):
    units = num & 15
    tens = (num>>4)*10
    return tens+units


#i2c.write_byte_data(rtc_address, 0, 12)

#print(i2c.read_byte_data(rtc_address, 0x0))
#print(i2c.read_byte_data(rtc_address, 0x7))
#print(i2c.read_byte_data(rtc_address, 0x3))
rtc_init()
rtc_set_time(11, 48, 20)


i2c.write_byte_data(rtc_address, 0x8, (1<<7)) #behind
for i in range(0,10000):
    rtc_read_time()
    time.sleep(0.01)



i2c.close()
