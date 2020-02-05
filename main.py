#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase


from pybricks.ev3devio import Ev3devSensor
import utime
import ev3dev2
from ev3dev2.port import LegoPort
from pybricks.robotics import DriveBase

class mySensor(Ev3devSensor):
    _ev3dev_driver_name = "ev3-analog-01"
    def readvalue(self):
        self._mode('ANALOG')
        return self._value(0)

left = Motor(Port.A)
right = Motor(Port.C)
wheelDiam =  56
wheelSpace = 180
tail = Motor(Port.B)

robot = DriveBase(left, right, wheelDiam, wheelSpace)
#kd = .9
kd = .04
# Write your program here

def adjustMovement(sensval1 ,sensval2):
    differ = sensval1 - sensval2 + 1
    speedadjust = abs(differ/36)
    #print(differ, ", TURNING: ", differ*kd/(1+differ), ", SPEED: ", -40/speedadjust)
    #robot.drive(-40/speedadjust, differ*kd/(1+differ))
    print(differ, ", TURNING: ", differ*kd, ", SPEED: ", -40/speedadjust)
    robot.drive(-40/speedadjust, differ*kd)

""" def adjustMovement(sensval1 ,sensval2, differ):
    differNew = (differ + (sensval1 - sensval2)*kd)
    speedadjust = (sensval1 - sensval2)*kd
    print(differ, ", TURNING: ", kd*differNew, ", SPEED: ", -40/(abs(speedadjust)))
    robot.drive(-40/(abs(speedadjust)), kd*differNew)
    return differNew """


def main():
    sens1 = LegoPort(address = 'ev3-ports:in3')
    sens1.mode = 'ev3-analog'
    utime.sleep(0.5)
    sensor_left = mySensor(Port.S2)
    sensor_right = mySensor(Port.S4)
    i = 1
    tot = 0
    while i < 5:
        val1 = sensor_left.readvalue()
        val2 = sensor_right.readvalue()
        summer = val1 + val2
        avg = summer/2
        tot = avg + tot
        i = i + 1
    errleft = sensor_left.readvalue() - tot/2
    errright = sensor_right.readvalue() - tot/2
    tail.dc(60)
    # differ = 1
    # differ1 =adjustMovement(val1,val2, differ)
    while True:
        val1 = sensor_left.readvalue()
        val2 = sensor_right.readvalue()
        #print(val1 , ',', val2)
        adjustMovement(val1 - errleft,val2 - errright)
        

main()