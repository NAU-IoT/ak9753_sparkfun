#! /usr/bin/env python3

from smbus import SMBus
import RPi.GPIO as GPIO


import time
import random


class AK9753():
	
	class Registers():
		WIA1 = 0x0
		WIA2 = 0x1
		INFO1 = 0x2
		INFO2 = 0x3
		INTST = 0x4
		ST1 = 0x5
		IR1L = 0x6
		IR1H = 0x7
		IR2L = 0x8
		IR2H = 0x9
		IR3L = 0xA
		IR3H = 0xB
		IR4L = 0xC
		IR4H = 0xD
		TMPL = 0xE
		TMPH = 0xF
		ST2 = 0x10
		ETH13H_L = 0x11
		ETH13H_H = 0x12
		ETH13L_L = 0x13
		ETH13L_H = 0x14
		ETH24H_L = 0x15
		ETH24H_H = 0x16
		ETH24L_L = 0x17
		ETH24L_H = 0x18
		EHYS13 = 0x19
		EHYS24 = 0x1A
		EINTEN = 0x1B
		ECNTL1 = 0x1C
		CNTL2 = 0x1D
	
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
		ECNTL1 = self.read_register(self.Registers.ECNTL1)
		mask = 0b11111000 + emode_val
		ECNTL1 = ECNTL1 & mask
		readback = self.write_register(self.Registers.ECNTL1, ECNTL1)
		
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
		


	def setPower(self, powerState):
		if self.powerPin is None:
			return #TODO: this should throw an exception?
			
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(self.powerPin, GPIO.OUT)
		
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
	hps = AK9753(i2c_bus, i2c_address = 0x64, powerPin = 11)
	hps.setPower(True)
	
	hps.setSingleShotMode()
	
	
	
	
	


if __name__ == "__main__":
	main()
	GPIO.cleanup()
	
