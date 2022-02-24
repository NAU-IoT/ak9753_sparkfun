#! /usr/bin/env python3

"""
Library for interfacing to the Sparkfun breakout board for the AK9753
four-channel PIR sensor.

See datasheet: https://cdn.sparkfun.com/assets/6/7/9/8/e/AK9753_DS.pdf
"""

from smbus import SMBus
import RPi.GPIO as GPIO

import datetime
import time
import random


# DEMO IMPORTS
#import matplotlib.pyplot as plt
#import matplotlib.animation as animation
#from matplotlib import style



class AK9753():
    """
    Class representing the AK9753. Generally all interfacing should be 
    done through the class."""

    class Registers():
        """
        Defines the address of user-accessible registers on the AK9753.
        Do not change these values.
        """
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

    def __init__(self, i2c_bus = 1, i2c_address = 0x64,
                 powerPin = None, intPin = None):
        """
        Initializes the sensor object. All parameters are optional; generally the default settings
        will work unless you need to hard-reset the AK9753 or you need hardware interrupts.

        Parameters:
            i2c_bus: Index of the Pi's I2C bus. This will always be 1 except for very old Pis.
            i2c_address: the I2C slave address of the AK9753.  The factory default is 0x64.
            powerPin: the Raspberry Pi PHYSICAL pin number supplying +3.3V to the AK9753.
            intPin: the Raspberry Pi PHYSICAL pin number for detecting interrupts from the AK9753.
        """

        GPIO.setmode(GPIO.BOARD) #use physical pin numbers

        self.i2c_bus = SMBus(i2c_bus)
        self.i2c_address = i2c_address
        self.powerPin = powerPin
        self.intPin = intPin

        if intPin:
            GPIO.setup(intPin, GPIO.IN)

    def readIntPin(self):
        """Returns the state of the pin connected to the AK9753's interrupt output.
        Returns False when the interrupt is active."""
        if self.intPin is None:
            raise AttributeError('No interrupt pin defined for this AK9753.')
        return GPIO.input(self.intPin)


    def powerOn(self):
        """Performs the example power-on procedure found in the AK9753 datasheet.
        Returns nothing."""
        self.performSoftReset()
        self.setDefaultMode()
        self.setInterruptSource()


    def getAllMeasurements(self):
        """Reads all the measurement output registers.  Returns a dictionary of useful values."""

        #sequentially read all the measurement value registers
        ST1 = self.read_register(self.Registers.ST1)
        IR1L = self.read_register()
        IR1H = self.read_register()
        IR2L = self.read_register()
        IR2H = self.read_register()
        IR3L = self.read_register()
        IR3H = self.read_register()
        IR4L = self.read_register()
        IR4H = self.read_register()
        TMPL = self.read_register()
        TMPH = self.read_register()
        ST2 = self.read_register() # reading ST2 tells the AK9753 that we're done reading

        IR1 = self.IRBytesToValue(lowByte = IR1L, highByte = IR1H)
        IR2 = self.IRBytesToValue(lowByte = IR2L, highByte = IR2H)
        IR3 = self.IRBytesToValue(lowByte = IR3L, highByte = IR3H)
        IR4 = self.IRBytesToValue(lowByte = IR4L, highByte = IR4H)

        TMP = self.TemperatureBytesToValue(lowByte = TMPL, highByte = TMPH)

        dict = {
                'ST1':ST1,
                'IR':[IR1, IR2, IR3, IR4],
                'tempC':TMP,
                'ST2':ST2
                }


        return dict

    def TemperatureBytesToValue(self, lowByte, highByte):
        """Takes the two-byte temperature measurement from the AK9753.  Returns the Celsius
        temperature indicated by those bytes using linear interpolation; see datasheet page 35."""

        val = (highByte * 256) + lowByte
        if val > 32767:
            val = ~val #flip all bits
            val = val & 0xFFFF #we only have two bytes, mask out everything else
            val = (val + 1)*-1

        # linear interpolation: 17792 ~ -18048  ==> 60*C ~ -10*C
        rangeDiff = 18048 + 17792
        valDiff = val + 18048

        tempRangeDiff = 70

        frac = valDiff / rangeDiff

        temp = -10 + (tempRangeDiff * frac)

        return temp

    def IRBytesToValue(self, lowByte, highByte):
        """Converts the two-byte IR measurement from a channel into a usable signed number.  Returns
        the actual signed value indicated by the register bytes. You should not need to call this
        externally."""
        val = (highByte * 256) + lowByte
        if val > 32767:
            val = ~val #flip all bits
            val = val & 0xFFFF #we only have two bytes, mask out everything else
            val = (val + 1)*-1
        return val

    def getIRValue(self, IRnum, terminate = True):
        """Returns the IR value from one specific channel.  Generally better to use the
        getAllMeasurements() function. 

        Parameters:
            IRNum: channel to read (1, 2, 3, or 4)
            terminate: whether to read the ST2 register after getting the IR value.
                       This tells the AK9753 that we're done reading and it's okay
                       to start collecting more data.


        """

        # don't try to read things other than IR values
        if type(IRnum) is not int:
            raise ValueError(f"IR channel must be an integer (got {IRnum})")
        if IRnum < 1 or IRnum > 4:
            raise ValueError(f"IR channel can be 1, 2, 3, 4 (got {IRnum})")

        lowByteAddress = 0x4 + 2*IRnum
        highByteAddress = lowByteAddress + 1
        lowByte = self.read_register(lowByteAddress)
        highByte = self.read_register(highByteAddress)

        # tell the AK9753 we're done reading, unless terminate = False was passed in
        if terminate:
            self.read_register(self.ST2)

        val = IRBytesToValue(lowByte = lowByte, highByte = highByte)

        return val


    def performSoftReset(self):
        """Commands the AK9753 to reset."""
        self.write_register(0x1d, 0x01, verify = False)

    def setDefaultMode(self):
        """Sets the AK9753 to its default mode."""
        self.write_register(0x1c, 0x0c, verify = False)

    def setInterruptSource(self, source = None):
        if source == None:
            self.write_register(0x1b, 0x01, verify = False)


    def setStandbyMode(self):
        """Stops measurements."""
        self.write_emode(0b000)

    def setSingleShotMode(self):
        """Tells the AK9753 to take a single measurement.  Sensor automatically returns to standby mode after measuring."""
        self.write_emode(0b010)

    def setEEPROMAccessMode(self):
        #TODO: implement this
        raise NotImplementedError("EEPROM Access Mode not yet implemented")

    def setContinuousMode0(self):
        """Continually take measurements as fast as the low pass filter settings allow."""
        self.write_emode(0b100)

    def setContinuousMode1(self):
        """Repeatedly take measurements; wait 1 measurement time between measurements."""
        self.write_emode(0b101)

    def setContinuousMode2(self):
        """Repeatedly take measurements; wait 3 measurement times between measurements."""
        self.write_emode(0b110)

    def setContinuousMode3(self):
        """Repeatedly take measurements; wait 7 measurement times between measurements."""
        self.write_emode(0b111)

    def isDataReady(self):
        """Checks whether a measurement is available.  Returns True if new data is ready."""
        ST1_val = self.read_register(self.Registers.ST1)
        DRDY = bool(ST1_val & 1) #get bit 0
        return DRDY

    def setFilterCutoffFrequency(self, frequency):
        """
        Sets the EFC filter cutoff frequency. This will be the measurement frequency in
        Continuous Mode 0.  Continuous Modes 1-3 wait a number of measurement times
        between each measurement.  See datasheet pg 39.

        Allowed values: 0.3, 0.6, 1.1, 2.2, 4.4, 8.8 (Hz)

        """

        allowedFrequencies = [0.3, 0.6, 1.1, 2.2, 4.4, 8.8]

        try:
            idx = allowedFrequences.index(frequency)
        except ValueError:
            raise ValueError("Invalid cutoff frequency.  Allowed values: 0.3, 0.6, 1.1, 2.2, 4.4, 8.8 (Hz).")


    def write_efc(self, efc_val):
        """Writes the EFC bits to the ECNTL1 register.  Not to be called directly; use setFilterCutoffFrequency()."""

        if type(efc_val) != int:
            raise ValueError("EFC bits must be an integer.")

        if (efc_val < 0) or (efc_val > 5):
            raise ValueError(f"Got EFC bits = {efc_val}.  Valid range is 0 to 5.")

        # get the value from the ECNTL1 register
        ECNTL1 = self.read_register(self.Registers.ECNTL1)

        # clear the EFC field
        mask = 0b11000111
        ECNTL1 = ECNTL & mask

        # move the EFC value into the right position
        efc_bits = efc_val * 8

        # set the EFC field
        ECNTL1 = ECNTL1 + efc_bits

        # write the register
        readback = self.write_register(self.Registers.ECNTL1, ECNTL1)

        if readback != ECNTL1:
            raise RuntimeError('Failed to set EFC cutoff frequency.')




    def write_emode(self, emode_val):
        """Sets the measurement mode.  Not intended to be called directly."""
        ECNTL1 = self.read_register(self.Registers.ECNTL1)
        mask = 0b11111000 + emode_val
        ECNTL1 = ECNTL1 & mask
        readback = self.write_register(self.Registers.ECNTL1, ECNTL1)

        if readback != ECNTL1:
            raise RuntimeError('Failed to switch AK9753 measurement modes.')



    def write_register(self, register, value, verify = True):
        """Writes any writable register on the AK9753. By default, verifies that the data was
        written correctly.  Generally you should leave the default in, unless this was a dummy
        write to set the address for an impending read operation."""

        write_result = self.i2c_bus.write_byte_data(self.i2c_address, register, value)

        if verify:
            readback = self.read_register(register)
            if readback != value:
                raise RuntimeError("Failed to write to AK9753 register {register}.")



    def read_register(self, register = None, terminate = True):
        """Reads an arbitrary register from the AK9753. If no register address is specified,
        reads from the current autoincrement register.  Returns register value.

        Parameter:
            terminate: When True, read from ST2 register after reading the specified
                       register.  When the specified register is a data output register,
                       ST2 must be read in order for the AK9753 to continue producing
                       data.  Set this to False if you will be reading other registers
                       immediately and need the measurement data to remain static. """

        if type(register) not in [None, int]:
            raise TypeError(f"Register address must be an integer (got {register})")

        if register is not None:
            #dummy write: sets register address to read from
            self.i2c_bus.write_byte(self.i2c_address, register)

        #now do the read
        read_result = self.i2c_bus.read_byte(self.i2c_address)

        # allow termination just in case we're reading from a data output register
        return read_result



    def setPower(self, powerState):
        """Turns power on or off to the AK9753."""

        if self.powerPin is None:
            raise AttributeError('No AK9753 power pin specified; cannot set power state.')

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.powerPin, GPIO.OUT)

        # if we are bringing power on (and was off previously)
        # then we need to make sure to wait for the AK9753 to
        # finish its power-on cycle before trying to do anything with it
        needToWait = (powerState and not(GPIO.input(self.powerPin)))

        # set the power state
        GPIO.output(self.powerPin, powerState)

        # wait if necessary
        if needToWait:
            time.sleep(.05)


def demo():


    SERIES_LENGTH = 7
    NX_THRESHOLD = 30
    INITIAL_PEOPLE = 2



    #i2c_bus = SMBus(1) #object representing our I2C bus
                        # uncomment and change to SMBus(0) for very old Pis

    hps = AK9753(i2c_address = 0x64, powerPin = 11, intPin = 7)
    hps.setPower(True)

    hps.examplePowerOn()


    log['t'] = []
    for n in range(1,5):
        log[n] = []

    startTime = None

    currentIR = [0, 0, 0, 0]
    last = [0, 0, 0, 0]
    d1 = [0, 0, 0, 0]
    lastD1 = [0, 0, 0, 0]
    d2 = [0, 0, 0, 0]
    series = []
    eventStr = ""

    people = INITIAL_PEOPLE

    stable = False

    while True:
        while True:
            time.sleep(.1)
            if not hps.readIntPin(): #wait until interrupt goes low
                break

        lastIR = currentIR


        meas = hps.getAllMeasurements()
        currentIR = meas['IR']
        tempC = meas['tempC']

        for idx in range(0,4):
            lastD1[idx] = d1[idx]
            d1[idx] = currentIR[idx] - lastIR[idx]
            d2[idx] = d1[idx] - lastD1[idx]

        d2_1_3 = d2[0] - d2[2]

        series.append(d2_1_3)
        if len(series) > SERIES_LENGTH:
            series.pop(0)


        maximum = max(series)
        minimum = min(series)

        if not stable:
            if maximum < NX_THRESHOLD and minimum > (NX_THRESHOLD * -1):
                stable = True
            else:
                continue

        if maximum > NX_THRESHOLD and minimum < (NX_THRESHOLD * -1) and len(series) == SERIES_LENGTH:
            maxIdx = series.index(maximum)
            minIdx = series.index(minimum)

            if eventStr == "":
                eventIsNew = True
            else:
                eventIsNew = False

            eventStr = "EVENT"
            #if series[SERIES_LENGTH-2] > NX_THRESHOLD:
            if maxIdx < minIdx:
                eventStr = "ENTRY"
                if eventIsNew:
                    people += 1
                    eventStr += f'     {str(people)}'
            #elif series[SERIES_LENGTH-2] < NX_THRESHOLD * -1:
            elif minIdx < maxIdx:
                eventStr = "EXIT"
                if eventIsNew:
                    people -= 1
                    eventStr += f'     {str(people)}'
        else:
            eventStr = ""

        #print(last, current, d1, '\t\t\t', d2)
        print(f"\t\t IR1'' - IR3'':  {d2_1_3}\t\tIR1'':  {d2[1]}   \t\tIR3'':  {d2[3]}\t\t{eventStr}    \t\t{tempC}*C")


        now = datetime.datetime.now()
        if startTime == None:
            startTime = now
        t = (now - startTime).total_seconds()




def main():

    demo()



if __name__ == "__main__":

    main()
    GPIO.cleanup()

