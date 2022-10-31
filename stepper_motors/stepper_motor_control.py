'''
Basic script to run a bipolar stepper motor with 200 steps/rev.

Relies on RpiMotorLib. Install found here:
https://github.com/gavinlyonsrepo/RpiMotorLib

Nice wiring tutorial here:
https://github.com/gavinlyonsrepo/RpiMotorLib/blob/master/Documentation/Nema11L298N.md

L298 stepper tester file:
https://github.com/gavinlyonsrepo/RpiMotorLib/blob/master/test/L298_Step_Test.py
'''

import time 
import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib


BLACK_PIN = 7
GREEN_PIN = 11
RED_PIN = 13
BLUE_PIN = 15
GpioPins = [BLACK_PIN, GREEN_PIN, RED_PIN, BLUE_PIN]


motor_one = RpiMotorLib.BYJMotor("MyMotorOne", "Nema")


def one_phase(self):
	'''
	Activates the motor using one phase.
	'''
	pass


# More torque
def two_phase(self):
	'''
	Activates the motor using both phases.
	'''
	pass


# More precise, twice as many steps
def half_step(self, step_delay, steps, counterclockwise):
	# Arguments for motor run function
    # (GPIOPins, stepdelay, steps, counterclockwise, verbose, steptype, initdelay)
    motor_one.motor_run(GpioPins, step_delay, steps, counterclockwise, True, "half", 1)
	pass


half_step(0.05, 25, True)
GPIO.cleanup() # Optional
exit()
