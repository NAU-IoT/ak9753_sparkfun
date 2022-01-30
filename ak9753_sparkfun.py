#! /usr/bin/env python3

from smbus import SMBus
import RPi.GPIO as GPIO

from enum import Enum
import time
import random


HPS_ADDR = 0x64
HPS_PWR_PIN = 11 #physical pin

class AK9753():
	
	def __init__(self, i2c_bus, i2c_address, 
				 powerPin = None, intPin = None):
		self.i2c_bus = i2c_bus
		self.i2c_address = i2c_address
		self.powerPin = powerPin
		self.intPin = intPin
	
	def setStandbyMode(self):
		self.write_emode(0)
	
	def setSingleShotMode(self):
		self.write_emode(2)
	
	
	def write_emode(self, emode_val):
		#todo: make this less stupid, maybe store the EEPMODE/EFC values
		ECNTL1 = self.read_register(0x1c)
		mask = 0b11111000 + emode_val
		ECNTL1 = ECNTL1 & mask
		readback = self.write_register(0x1c, ECNTL1)
		
		if readback == ECNTL1:
			print(f'Mode switch OK, EMODE now {emode_val}\n')
		else:
			print('Mode switch failed\n')
		
		

		


	def write_register(self, register, value, verify = True):
		#writes any write-enabled register on the AK9753
		
		write_result = self.i2c_bus.write_byte_data(self.i2c_address, register, value)
		
		if verify:
			readback = self.read_register(register)
			return readback
		else:
			return None



	def read_register(self, register):
		#reads any register from the AK9753
		
		#dummy write: sets register address to read from
		self.i2c_bus.write_byte(self.i2c_address, register)
		
		#now do the read
		read_result = self.i2c_bus.read_byte(self.i2c_address)
		return read_result
		


	def HPS_set_power(self, powerState):
		if self.powerPin is None:
			return #TODO: this should throw an exception?
			
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(powerPin, GPIO.OUT)
		
		# if we are bringing power on (and was off previously)
		# then we need to make sure to wait for the AK9753 to 
		# finish its power-on cycle before trying to do anything with it
		needToWait = (powerState and not(GPIO.input(self.powerPin))) 
		
		# turn the power on	
		GPIO.output(self.powerPin, powerState)
		
		# wait if necessary
		if needToWait:
			time.sleep(.05) 
		
	



def main():
	#this is just for testing/demonstration purposes
	
	
	i2c_bus = SMBus(1) #object representing our I2C bus
	hps = AK9753(i2c_bus, HPS_ADDR)
	
	hps.setSingleShotMode()
	
	
	
	
	


if __name__ == "__main__":
	main()
	GPIO.cleanup()
	
