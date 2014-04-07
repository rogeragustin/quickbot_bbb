#!/usr/bin/python
import math
import serial

def parseEncoderBuffer(side):
    line = encoderSerial[side].readline()
    print line
    msgStartInd  = line.find("D")
    if msgStartInd == 0:
        val = convertHEXtoDEC(line[1:9], 8)
        if not math.isnan(val):
            print val

    line = encoderSerial[side].readline()
    print line
    msgStartInd = line.find("V")
    if msgStartInd == 0:
        vel = convertHEXtoDEC(line[1:5], 4)
        if not math.isnan(vel):
            print vel
        encoderSerial[side].flushInput()

def convertHEXtoDEC(hexString, N):
    # Return 2's compliment of hexString
    for hexChar in hexString:
        asciiNum = ord(hexChar)
        if not ((asciiNum >= 48 and asciiNum <= 57) or \
             (asciiNum >= 65 and asciiNum <= 70) or \
             (asciiNum >= 97 and asciiNum <= 102)):
             val = float('nan')
             return val

    if len(hexString) == N:
        val = int(hexString, 16)
        bits = 4*len(hexString)
        if  (val & (1<<(bits-1))) != 0:
            val = val - (1<<bits)
        return val


LEFT = 0
RIGHT = 1
encoderSerial = (serial.Serial(port = "/dev/ttyO1", baudrate = 38400, timeout = .1), \
                 serial.Serial(port = "/dev/ttyO2", baudrate = 38400, timeout = .1))

encoderSerial[LEFT].write('V')

runFlag = True
while runFlag == True:
    while (encoderSerial[LEFT].inWaiting() > 16):
        print 'In 1: ' + str(encoderSerial[LEFT].inWaiting())
        parseEncoderBuffer(LEFT)
        print 'In 2: ' + str(encoderSerial[LEFT].inWaiting())
    while (encoderSerial[RIGHT].inWaiting() > 0):
        parseEncoderBuffer(RIGHT)