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


# Black, Green, Red, Blue
GpioPinsOne = [7, 11, 13, 15]
GpioPinsTwo = [12, 16, 18, 22]
GpioPinsThree = [29, 31, 33, 35]
GpioPinsFour = [32, 36, 38, 40]


front_left_motor = RpiMotorLib.BYJMotor("MotorOne", "Nema")
front_right_motor = RpiMotorLib.BYJMotor("MotorTwo", "Nema")
back_left_motor = RpiMotorLib.BYJMotor("MotorThree", "Nema")
back_right_motor = RpiMotorLib.BYJMotor("MotorFour", "Nema")


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
def half_step(self, motor, step_delay, steps, counterclockwise):
	# Arguments for motor run function
    # (GPIOPins, stepdelay, steps, counterclockwise, verbose, steptype, initdelay)
    motor.motor_run(GpioPinsOne, step_delay, steps, counterclockwise, True, "half", 1)
	pass


def move_north(self, step_delay, steps):
	'''
	Moves all four wheels forward.
	'''
	half_step(front_left_motor, step_delay, steps, True)
	half_step(front_right_motor, step_delay, steps, False)
	half_step(back_left_motor, step_delay, steps, True)
	half_step(back_right_motor, step_delay, steps, False)


def move_west(self):
	'''
	front left: backward
	front right: forward
	back left: forward
	back right: backward
	'''
	half_step(front_left_motor, step_delay, steps, False)
	half_step(front_right_motor, step_delay, steps, False)
	half_step(back_left_motor, step_delay, steps, True)
	half_step(back_right_motor, step_delay, steps, True)


def move_east(self, step_delay, steps):
	'''
	front left: forward
	front right: backward
	back left: backward
	back right: forward
	'''
	half_step(front_left_motor, step_delay, steps, True)
	half_step(front_right_motor, step_delay, steps, True)
	half_step(back_left_motor, step_delay, steps, False)
	half_step(back_right_motor, step_delay, steps, False)


def move_south(self, step_delay, steps):
	'''
	Moves all four wheels backward.
	'''
	half_step(front_left_motor, step_delay, steps, False)
	half_step(front_right_motor, step_delay, steps, True)
	half_step(back_left_motor, step_delay, steps, False)
	half_step(back_right_motor, step_delay, steps, True)




move_north(0.05, 25)
GPIO.cleanup() # Optional
exit()
