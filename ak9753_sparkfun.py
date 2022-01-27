#! /usr/bin/env python3



from smbus import SMBus
import RPi.GPIO as GPIO
import time
import random


HPS_ADDR = 0x64
HPS_PWR_PIN = 11 #physical pin


def write_register(i2c_bus, i2c_address, register, value, verify = True):
	#writes any write-enabled register on the AK9753
	
	write_result = i2c_bus.write_byte_data(i2c_address, register, value)
	
	if verify:
		readback = read_register(i2c_bus, i2c_address, register)
		return readback
	else:
		return None



def read_register(i2c_bus, i2c_address, register):
	#reads any register from the AK9753
	
	#dummy write: sets register address to read from
	i2c_bus.write_byte(i2c_address, register)
	
	#now do the read
	read_result = i2c_bus.read_byte(i2c_address)
	return read_result
		


def HPS_set_power(powerPin, powerState):
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(powerPin, GPIO.OUT)
	
	# if we are bringing power on (and was off previously)
	# then we need to make sure to wait for the AK9753 to 
	# finish its power-on cycle before trying to do anything with it
	needToWait = (powerState and not(GPIO.input(powerPin))) 
	
	# turn the power on	
	GPIO.output(powerPin, powerState)
	
	# wait if necessary
	if needToWait:
		time.sleep(.05) 
		
	



def main():
	i2c = SMBus(1) #object representing our I2C bus
	HPS_set_power(HPS_PWR_PIN, False)
	time.sleep(1)
	HPS_set_power(HPS_PWR_PIN, True)
	
	
	
	read_result = read_register(i2c_bus = i2c,
								i2c_address = HPS_ADDR,
								register = 0x1b)
								
	print('EINTEN = ',read_result, f'({bin(read_result)})')                                                                 
	print('Attempting to write to EINTEN...')
	val = random.randint(0,63)
	print(val+192)
	write_register(i2c, HPS_ADDR, 0x1b, val)
	
	
	read_result = read_register(i2c_bus = i2c,
								i2c_address = HPS_ADDR,
								register = 0x1b)							
	print('EINTEN = ',read_result, f'({bin(read_result)})')
	
	


if __name__ == "__main__":
	main()
	GPIO.cleanup()
	
