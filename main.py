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

#left = Motor(Port.A)
#right = Motor(Port.B)
#wheelDiam =  56
#wheelSpace = 180

#robot = DriveBase(left, right, wheelDiam, wheelSpace)
kd = .9
# Write your program here

def adjustMovement(sensval1 ,sensval2):
    differ = sensval1 - sensval2
    #robot.drive(40, kd*differ)


def main():
    sens1 = LegoPort(address = 'ev3-ports:in2')
    sens2 = LegoPort(address = 'ev3-ports:in3')
    sens1.mode = 'ev3-analog'
    sens2.mode = 'ev3-analog'
    utime.sleep(0.5)
    sensor_left = mySensor(Port.S2)
    sensor_right = mySensor(Port.S3)
    val1 = sensor_left.readvalue()
    val2 = sensor_right.readvalue()
    print(val1 , ',', val2)
    #adjustMovement(val1,val2)
    while True:
        print(sensor_left.readvalue())
        wait(200)
    

main()