#!/usr/bin/python
"""
@brief QuickBot class for Beaglebone Black

@author Rowland O'Flaherty (rowlandoflaherty.com)
@date 02/07/2014
@version: 1.0
@copyright: Copyright (C) 2014, Georgia Tech Research Corporation
see the LICENSE file included with this software (see LINENSE file)
"""

from __future__ import division
import sys
import time
import math
import re
import serial
import socket
import threading
import numpy as np

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC

# Constants
LEFT = 0
RIGHT = 1
MIN = 0
MAX = 1

DEBUG = False

## Tic toc constants
TICTOC_START = 0
TICTOC_COUNT = 0
TICTOC_MEAN = 0
TICTOC_MAX = -float('inf')
TICTOC_MIN = float('inf')

ADCTIME = 0.002
ADC_LOCK = threading.Lock()

## Run variables
RUN_FLAG = True
RUN_FLAG_LOCK = threading.Lock()


class QuickBot():
    """The QuickBot Class"""

    # === Class Properties ===
    # Parameters
    sampleTime = 20.0 / 1000.0

    # Pins
    ledPin = 'USR1'

    # Motor Pins -- (LEFT, RIGHT)
    dir1Pin = ('P8_14', 'P8_12')
    dir2Pin = ('P8_16', 'P8_10')
    pwmPin = ('P9_16', 'P9_14')

    # ADC Pins
    IRPin = ('P9_38', 'P9_40', 'P9_36', 'P9_35', 'P9_33')
#     encPosPin = ('P9_13', 'P9_12')
#     encDirPin = ('P9_15', 'P9_11')

    # Serial
    # TTYO1: Rx=P9_26  Tx=P9_24
    # TTYO2: Rx=P9_22  Tx=P9_21
    # TTYO4: Rx=P9_11  Tx=P9_13
    # TTYO5: Rx=P9_38  Tx=P9_38
    encoderSerial = (serial.Serial(port = '/dev/ttyO1', baudrate = 38400, timeout = .1), \
                     serial.Serial(port = '/dev/ttyO2', baudrate = 38400, timeout = .1))


    # Constraints
    pwmLimits = [-100, 100]  # [min, max]
    
    # Wheel parameter
    ticksPerTurn = 128  # Number of ticks on encoder disc
    wheelRadius = (58.7 / 2.0) / 1000.0  # Radius of wheel in meters

    # Other variables
    ledFlag = True
    cmdBuffer = ''
    encoderBuffer = ['', '']

    # UDP variables
    baseIP = '192.168.7.1'
    robotIP = '192.168.7.2'
    port = 5005
    robotSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robotSocket.setblocking(False)

    # === Class Methods ===
    # Constructor
    def __init__(self, baseIP, robotIP):

        # Initialize GPIO pins
        for side in range(0,2):
            GPIO.setup(self.dir1Pin[side], GPIO.OUT)
            GPIO.setup(self.dir2Pin[side], GPIO.OUT)
            
        GPIO.setup(self.ledPin, GPIO.OUT)

        # Initialize PWM pins: PWM.start(channel, duty, freq=2000, polarity=0)
        PWM.start(self.pwmPin[LEFT], 0)
        PWM.start(self.pwmPin[RIGHT], 0)
        
        # State PWM -- (LEFT, RIGHT)
        self.pwm = [0, 0]
    
        # State IR
        self.nIR = len(self.IRPin)
        self.IRVal = self.nIR*[0.0]
    
        # State Encoder
        self.encDir = [1, -1]      # Last encoder direction
        self.encPos = [0, 0]      # Last encoder tick position
        self.encVel = [0.0, 0.0]  # Last encoder tick velocity
        self.encPosOffset = [0, 0] # Offset from raw encoder tick

        # Set motor speed to 0
        self.setPWM(self.pwm)

        # Initialize ADC
        ADC.setup()
        
        # Initialize command parsing thread
        self.cmdParsingThread = threading.Thread(target = parseCmd, args = (self, ))
        self.cmdParsingThread.daemon = True
        
        # Initialize IR thread
        self.IRThread = threading.Thread(target = readIR, args = (self, ))
        self.IRThread.daemon = True
         
        # Initialize encoder threads
        self.encDirThread = 2*[None]
        self.encPosThread = 2*[None]        
        self.encVelThread = 2*[None]
        for side in range(0, 2):
            self.encPosThread[side] = threading.Thread(target = readEncPos, args = (self, side))
            self.encPosThread[side].daemon = True
            
        # Set IP addresses
        self.baseIP = baseIP
        self.robotIP = robotIP
        self.robotSocket.bind((self.robotIP, self.port))
        
    # Getters and Setters
    def setPWM(self, pwm):
        # [leftSpeed, rightSpeed]: 0 is off, caps at min and max values

        self.pwm[LEFT] = min(
            max(pwm[LEFT], self.pwmLimits[MIN]), self.pwmLimits[MAX])
        self.pwm[RIGHT] = min(
            max(pwm[RIGHT], self.pwmLimits[MIN]), self.pwmLimits[MAX])

        # Left motor
        if self.pwm[LEFT] > 0:
            GPIO.output(self.dir1Pin[LEFT], GPIO.LOW)
            GPIO.output(self.dir2Pin[LEFT], GPIO.HIGH)
            PWM.set_duty_cycle(self.pwmPin[LEFT], abs(self.pwm[LEFT]))
        elif self.pwm[LEFT] < 0:
            GPIO.output(self.dir1Pin[LEFT], GPIO.HIGH)
            GPIO.output(self.dir2Pin[LEFT], GPIO.LOW)
            PWM.set_duty_cycle(self.pwmPin[LEFT], abs(self.pwm[LEFT]))
        else:
            GPIO.output(self.dir1Pin[LEFT], GPIO.LOW)
            GPIO.output(self.dir2Pin[LEFT], GPIO.LOW)
            PWM.set_duty_cycle(self.pwmPin[LEFT], 0)

        # Right motor
        if self.pwm[RIGHT] > 0:
            GPIO.output(self.dir1Pin[RIGHT], GPIO.LOW)
            GPIO.output(self.dir2Pin[RIGHT], GPIO.HIGH)
            PWM.set_duty_cycle(self.pwmPin[RIGHT], abs(self.pwm[RIGHT]))
        elif self.pwm[RIGHT] < 0:
            GPIO.output(self.dir1Pin[RIGHT], GPIO.HIGH)
            GPIO.output(self.dir2Pin[RIGHT], GPIO.LOW)
            PWM.set_duty_cycle(self.pwmPin[RIGHT], abs(self.pwm[RIGHT]))
        else:
            GPIO.output(self.dir1Pin[RIGHT], GPIO.LOW)
            GPIO.output(self.dir2Pin[RIGHT], GPIO.LOW)
            PWM.set_duty_cycle(self.pwmPin[RIGHT], 0)

    # Methods
    def run(self):
        global RUN_FLAG
        
        # Start threads
        self.cmdParsingThread.start()
        self.IRThread.start()
        for side in range(0, 2):
            self.encPosThread[side].start()
        
        # Run loop
        self.calEncPos()
        while RUN_FLAG is True:
            self.update()

            # Flash BBB LED
            if self.ledFlag is True:
                self.ledFlag = False
                GPIO.output(self.ledPin, GPIO.HIGH)
            else:
                self.ledFlag = True
                GPIO.output(self.ledPin, GPIO.LOW)
#                 print '[' + ', '.join(map(str, self.getEncPos())) + ']'
            time.sleep(self.sampleTime)
        
        self.cleanup()
        return

    def cleanup(self):
        sys.stdout.write("Shutting down...")
        self.setPWM([0, 0])
        self.robotSocket.close()
        GPIO.cleanup()
        PWM.cleanup()
        if DEBUG:
            pass
            # tictocPrint()
            # self.writeBuffersToFile()
        sys.stdout.write("Done\n")
        
    def calEncPos(self):
        self.setPWM([100, 100])
        time.sleep(0.1)
        self.setPWM([0, 0])
        time.sleep(1.0)
        self.resetEncPos()

    def getEncPos(self):
        return [self.encPos[LEFT] - self.encPosOffset[LEFT], -1*(self.encPos[RIGHT] - self.encPosOffset[RIGHT])]
    
    def resetEncPos(self):
        self.encPosOffset[LEFT] = self.encPos[LEFT]
        self.encPosOffset[RIGHT] = self.encPos[RIGHT]
        
    def getPos(self):
        pos = [0.0, 0.0]
        encPos = self.getEncPos()
        for side in range(0,2):
            pos[side] = encPos[side] / self.ticksPerTurn * 2 * np.pi * self.wheelRadius
        return pos

    def update(self):
        pass
#         self.parseCmdBuffer()


    def writeBuffersToFile(self):
        pass
#         matrix = map(list, zip(*[self.encTimeRec[LEFT], self.encValRec[LEFT], self.encPWMRec[LEFT], self.encNNewRec[LEFT], \
#                                  self.encTickStateRec[LEFT], self.encPosRec[LEFT], self.encVelRec[LEFT], self.encThresholdRec[LEFT], \
#                                  self.encTimeRec[RIGHT], self.encValRec[RIGHT], self.encPWMRec[RIGHT], self.encNNewRec[RIGHT], \
#                                  self.encTickStateRec[RIGHT], self.encPosRec[RIGHT], self.encVelRec[RIGHT], self.encThresholdRec[RIGHT]]))
#         s = [[str(e) for e in row] for row in matrix]
#         lens = [len(max(col, key=len)) for col in zip(*s)]
#         fmt = '\t'.join('{{:{}}}'.format(x) for x in lens)
#         table = [fmt.format(*row) for row in s]
#         f = open('output.txt', 'w')
#         f.write('\n'.join(table))
#         f.close()
#         print "Wrote buffer to output.txt"

def parseCmd(self):
    global RUN_FLAG
    
    while RUN_FLAG:
        try:
            line = self.robotSocket.recv(1024)
        except socket.error as msg:
            continue
    
        self.cmdBuffer += line
    
        bufferPattern = r'\$[^\$\*]*?\*'  # String contained within $ and * symbols with no $ or * symbols in it
        bufferRegex = re.compile(bufferPattern)
        bufferResult = bufferRegex.search(self.cmdBuffer)
    
        if bufferResult:
            msg = bufferResult.group()
#             print msg
            self.cmdBuffer = ''
    
            msgPattern = r'\$(?P<CMD>[A-Z]{3,})(?P<SET>=?)(?P<QUERY>\??)(?(2)(?P<ARGS>.*)).*\*'
            msgRegex = re.compile(msgPattern)
            msgResult = msgRegex.search(msg)
    
            if msgResult.group('CMD') == 'CHECK':
                self.robotSocket.sendto('Hello from QuickBot\n',(self.baseIP, self.port))
    
            elif msgResult.group('CMD') == 'PWM':
                if msgResult.group('QUERY'):
                    self.robotSocket.sendto(str(self.pwm) + '\n',(self.baseIP, self.port))
    
                elif msgResult.group('SET') and msgResult.group('ARGS'):
                    args = msgResult.group('ARGS')
                    pwmArgPattern = r'(?P<LEFT>[-]?\d+),(?P<RIGHT>[-]?\d+)'
                    pwmRegex = re.compile(pwmArgPattern)
                    pwmResult = pwmRegex.match(args)
                    if pwmResult:
                        pwm = [int(pwmRegex.match(args).group('LEFT')),
                            int(pwmRegex.match(args).group('RIGHT'))]
                        self.setPWM(pwm)
    
            elif msgResult.group('CMD') == 'IRVAL':
                if msgResult.group('QUERY'):
                    reply = '[' + ', '.join(map(str, self.IRVal)) + ']'
                    print 'Sending: ' + reply
                    self.robotSocket.sendto(reply + '\n', (self.baseIP, self.port))
                    
            elif msgResult.group('CMD') == 'POS':
                if msgResult.group('QUERY'):
                    reply = '[' + ', '.join(map(str, self.getPos())) + ']'
                    print 'Sending: ' + reply
                    self.robotSocket.sendto(reply + '\n', (self.baseIP, self.port))
    
            elif msgResult.group('CMD') == 'ENPOS' or msgResult.group('CMD') == 'ENVAL':
                if msgResult.group('QUERY'):
                    reply = '[' + ', '.join(map(str, self.getEncPos())) + ']'
                    print 'Sending: ' + reply
                    self.robotSocket.sendto(reply + '\n', (self.baseIP, self.port))
    
            elif msgResult.group('CMD') == 'ENVEL':
                if msgResult.group('QUERY'):
                    reply = '[' + ', '.join(map(str, self.encVel)) + ']'
                    print 'Sending: ' + reply
                    self.robotSocket.sendto(reply + '\n', (self.baseIP, self.port))
    
            elif msgResult.group('CMD') == 'RESET':
                self.resetEncPos()
                print 'Encoder values reset to [' + ', '.join(map(str, self.encVel)) + ']'
    
            elif msgResult.group('CMD') == 'UPDATE':
                if msgResult.group('SET') and msgResult.group('ARGS'):
                    args = msgResult.group('ARGS')
                    pwmArgPattern = r'(?P<LEFT>[-]?\d+),(?P<RIGHT>[-]?\d+)'
                    pwmRegex = re.compile(pwmArgPattern)
                    pwmResult = pwmRegex.match(args)
                    if pwmResult:
                        pwm = [int(pwmRegex.match(args).group('LEFT')),
                            int(pwmRegex.match(args).group('RIGHT'))]
                        self.setPWM(pwm)
    
                    reply = '[' + ', '.join(map(str, self.encPos)) + ', ' \
                        + ', '.join(map(str, self.encVel)) + ']'
                    print 'Sending: ' + reply
                    self.robotSocket.sendto(reply + '\n', (self.baseIP, self.port))
    
            elif msgResult.group('CMD') == 'END':
                RUN_FLAG_LOCK.acquire()
                RUN_FLAG = False
                RUN_FLAG_LOCK.release()

def readIR(self):
    global RUN_FLAG
    
    while RUN_FLAG:
        for i in range(0,self.nIR):
            ADC_LOCK.acquire()
            self.IRVal[i] = ADC.read_raw(self.IRPin[i])
            time.sleep(ADCTIME)
            ADC_LOCK.release()
                       
                
def readEncPos(self, side):
    global RUN_FLAG
    sampleTime = (20.0 / 1000.0)
    
    while RUN_FLAG:
        parseEncoderBuffer(self, side)
        time.sleep(sampleTime)
            
    
def parseEncoderBuffer(self, side):
    encoderUpdateFlag = False

    bytesInWaiting = self.encoderSerial[side].inWaiting()

    if (bytesInWaiting > 0):
        self.encoderBuffer[side] += self.encoderSerial[side].read(bytesInWaiting)

        if len(self.encoderBuffer[side]) > 30:
            self.encoderBuffer[side] = self.encoderBuffer[side][-30:]

        if len(self.encoderBuffer[side]) >= 15:
            DPattern = r'D([0-9A-F]{8})'
            DRegex = re.compile(DPattern)
            DResult = DRegex.findall(self.encoderBuffer[side])
            if len(DResult) >= 1:
                val = convertHEXtoDEC(DResult[-1], 8)
                if not math.isnan(val):
                    self.encPos[side] = val
                    encoderUpdateFlag = True

            VPattern = r'V([0-9A-F]{4})'
            VRegex = re.compile(VPattern)
            VResult = VRegex.findall(self.encoderBuffer[side])
            if len(VResult) >= 1:
                vel = convertHEXtoDEC(VResult[-1], 4)
                if not math.isnan(vel):
                    self.encVel[side] = vel
                    encoderUpdateFlag = True

        return encoderUpdateFlag
    
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
            

def recursiveMeanVar(x, l, mu, sigma2):
    """
    This function calculates a new mean and variance given
    the current mean "mu", current variance "sigma2", current
    update count "l", and new samples "x"
    """

    m = np.size(x)
    n = l + m
    muPlus = l / n * mu + m / n * np.mean(x)
    if n > 1:
        sigma2Plus = 1/(n-1) * ((l-1)*sigma2 + (m-1)*np.var(x) + l*(mu - muPlus)**2 + m*(np.mean(x) - muPlus)**2)
    else:
        sigma2Plus = 0

    return (muPlus, sigma2Plus, n)

def operatingPoint(uStar, uStarThreshold):
    """
    This function returns the steady state tick velocity given some PWM input.

    uStar: PWM input.
    uStarThreshold: Threshold on the minimum magnitude of a PWM input value

    returns: omegaStar - steady state tick velocity
    """
    # Matlab code to find beta values
    # X = [40; 80; 100]; % Air Test
    # Y = [0.85; 2.144; 3.5];
    #
    # r = 0.0325; % Wheel radius
    # c = 2*pi*r;
    # X = [  70;   70;   70;   75;   75;   75;   80;   80;   80; 85;     85;   85;   90;   90;   90]; % Ground Test
    # Z = [4.25; 3.95; 4.23; 3.67; 3.53; 3.48; 3.19; 3.08; 2.93; 2.52; 2.59; 2.56; 1.99; 2.02; 2.04]; % Time to go 1 m
    # Y = 1./(Z*c);
    # H = [X ones(size(X))];
    # beta = H \ Y
    # beta = [0.0425, -0.9504] # Air Test Results
    beta = [0.0606, -3.1475] # Ground Test Results

    if np.abs(uStar) <= uStarThreshold:
        omegaStar = 0.0
    elif uStar > 0:
        omegaStar = beta[0]*uStar + beta[1]
    else:
        omegaStar = -1.0*(beta[0]*np.abs(uStar) + beta[1])

    return omegaStar


def kalman(x, P, Phi, H, W, V, z):
    """
    This function returns an optimal expected value of the state and covariance
    error matrix given an update and system parameters.

    x:   Estimate of state at time t-1.
    P:   Estimate of error covariance matrix at time t-1.
    Phi: Discrete time state transition matrix at time t-1.
    H:   Observation model matrix at time t.
    W:   Process noise covariance at time t-1.
    V:   Measurement noise covariance at time t.
    z:   Measurement at time t.

    returns: (x,P) tuple
    x: Updated estimate of state at time t.
    P: Updated estimate of error covariance matrix at time t.

    """
    x_p = Phi*x  # Prediction of estimated state vector
    P_p = Phi*P*Phi + W  # Prediction of error covariance matrix
    S = H*P_p*H + V  # Sum of error variances
    S_inv = 1/S  # Inverse of sum of error variances
    K = P_p*H*S_inv  # Kalman gain
    r = z - H*x_p  # Prediction residual
    w = -K*r  # Process error
    x = x_p - w  # Update estimated state vector
    v = z - H*x  # Measurement error
    if np.isnan(K*V):
        P = P_p
    else:
        P = (1 - K*H)*P_p*(1 - K*H) + K*V*K  # Updated error covariance matrix

    return (x, P)


def tic():
    global TICTOC_START
    TICTOC_START = time.time()


def toc(tictocName = 'toc', printFlag = True):
    global TICTOC_START
    global TICTOC_COUNT
    global TICTOC_MEAN
    global TICTOC_MAX
    global TICTOC_MIN

    tictocTime = time.time() - TICTOC_START
    TICTOC_COUNT = TICTOC_COUNT + 1
    TICTOC_MEAN = tictocTime / TICTOC_COUNT + TICTOC_MEAN * (TICTOC_COUNT-1) / TICTOC_COUNT
    TICTOC_MAX = max(TICTOC_MAX,tictocTime)
    TICTOC_MIN = min(TICTOC_MIN,tictocTime)

    if printFlag:
        print tictocName + " time: " + str(tictocTime)

def tictocPrint():
    global TICTOC_COUNT
    global TICTOC_MEAN
    global TICTOC_MAX
    global TICTOC_MIN

    print "Tic Toc Stats:"
    print "Count = " + str(TICTOC_COUNT)
    print "Mean = " + str(TICTOC_MEAN)
    print "Max = " + str(TICTOC_MAX)
    print "Min = " + str(TICTOC_MIN)


