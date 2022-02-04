#! /usr/bin/env python3

from smbus import SMBus
import RPi.GPIO as GPIO


import time
import random


# DEMO IMPORTS
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import datetime



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
        if intPin:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(intPin, GPIO.IN)

    def readIntPin(self):
        return GPIO.input(self.intPin)

    def examplePowerOn(self):
        self.performSoftReset()
        self.setDefaultMode()
        self.setInterruptSource()


    def getAllMeasurements(self):
        #TODO: refactor to use address auto-increment
        #TODO: get the temperature information
        INTST_val = self.read_register(self.Registers.INTST)
        ST1_val = self.read_register(self.Registers.ST1)
        IRValues = [None,
                    self.getIRValue(1),
                    self.getIRValue(2),
                    self.getIRValue(3),
                    self.getIRValue(4)]
        ST2_val = self.read_register(self.Registers.ST2)

        return IRValues


    def getIRValue(self, IRnum):
        #TODO: refactor to use address auto increment
        lowByteAddress = 0x4 + 2*IRnum
        highByteAddress = lowByteAddress + 1
        lowByte = self.read_register(lowByteAddress)
        highByte = self.read_register(highByteAddress)


        val = (highByte * 256) + lowByte
        if val > 32767:
            val = ~val #flip all bits
            val = val & 0xFFFF #mask off lower two bytes
            val = (val + 1)*-1


        return val


    def performSoftReset(self):
        self.write_register(0x1d, 0x01, verify = False)

    def setDefaultMode(self):
        self.write_register(0x1c, 0x0c, verify = False)

    def setInterruptSource(self, source = None):
        if source == None:
            self.write_register(0x1b, 0x01, verify = False)



    def setStandbyMode(self):
        self.write_emode(0b000)

    def setSingleShotMode(self):
        self.write_emode(0b010)

    def setEEPROMAccessMode(self):
        #TODO: implement this
        print("EEPROM Access Mode not yet implemented")

    def setContinuousMode0(self):
        self.write_emode(0b100)

    def setContinuousMode1(self):
        self.write_emode(0b101)

    def setContinuousMode2(self):
        self.write_emode(0b110)

    def setContinuousMode3(self):
        self.write_emode(0b111)

    def isDataReady(self):
        ST1_val = self.read_register(self.Registers.ST1)
        DRDY = bool(ST1_val & 1) #get bit 0
        return DRDY




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

def updatePlot(figure, data):


    ax1 = figure.add_subplot(3,1,1)
    ax2 = figure.add_subplot(3,1,2)
    ax3 = figure.add_subplot(3,1,3)

    ax1.cla()

    diffs = {}
    for n in range(1,5):
        diffs[n] = [0]

    for step in range(1, len(data['t'])):
        for ch in range(1,5):
            d = data[ch][step] - data[ch][step-1]
            diffs[ch].append(d)


    for ch in range(1,5):
        ax1.plot(data['t'], data[ch], label=f'Ch{ch} val')

    for ch in range(1,5):
        ax2.plot(data['t'], diffs[ch], label=f'Ch{ch} dt')

    d24 = [0]
    for step in range(1, len(data['t'])):
        d24.append(diffs[2][step] - diffs[4][step])

    ax3.plot(data['t'], d24, label = 'd2dt - d4dt')


    ax1.legend()
    ax2.legend()
    ax3.legend()


log = {}


def main():
    #this is just for testing/demonstration purposes


    #plt.rcdefaults()

    #figure = plt.figure()


    i2c_bus = SMBus(1) #object representing our I2C bus
    hps = AK9753(i2c_bus, i2c_address = 0x64, powerPin = 11, intPin = 7)
    hps.setPower(True)

    hps.examplePowerOn()


    log['t'] = []
    for n in range(1,5):
        log[n] = []


    startTime = None

    SERIES_LENGTH = 5
    NX_THRESHOLD = 20

    current = [0, 0, 0, 0]
    last = [0, 0, 0, 0]
    d1 = [0, 0, 0, 0]
    lastD1 = [0, 0, 0, 0]
    d2 = [0, 0, 0, 0]
    series = []
    eventStr = ""

    people = 3

    stable = False

    #int has gone low, see if data is ready
    #for count in range(1,50):
    while True:
        while True:
            time.sleep(.1)
            if not hps.readIntPin():
                break

        last = current


        current = hps.getAllMeasurements()
        current.pop(0) #throw away temperature measurement




        for idx in range(0,4):
            lastD1[idx] = d1[idx]
            d1[idx] = current[idx] - last[idx]
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
            if maxIdx > minIdx:
                eventStr = "ENTRY"
                if eventIsNew:
                    people += 1
                    eventStr += f'     {str(people)}'
            #elif series[SERIES_LENGTH-2] < NX_THRESHOLD * -1:
            elif minIdx > maxIdx:
                eventStr = "EXIT"
                if eventIsNew:
                    people -= 1
                    eventStr += f'     {str(people)}'
        else:
            eventStr = ""

        #print(last, current, d1, '\t\t\t', d2)
        print('\t\t', d2_1_3, '\t', d2[1], d2[3],  '\t\t', eventStr)

        now = datetime.datetime.now()
        if startTime == None:
            startTime = now
        t = (now - startTime).total_seconds()








    #updatePlot(figure, log)
    #plt.show()


if __name__ == "__main__":

    main()
    GPIO.cleanup()

