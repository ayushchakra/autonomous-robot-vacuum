'''
Helper functions for the stepper motors
'''

import RPi.GPIO as GPIO
import time



def _setup_pins(motor_pins):
    for pin in motor_pins:
        GPIO.setup(pin, GPIO.OUT)

# step 1 - high low high low
def _phase_one(motor_pins):
    GPIO.output(motor_pins[0], GPIO.HIGH)
    GPIO.output(motor_pins[1], GPIO.LOW)
    GPIO.output(motor_pins[2], GPIO.HIGH)
    GPIO.output(motor_pins[3], GPIO.LOW)

# step 2 - low high high low
def _phase_two(motor_pins):
    GPIO.output(motor_pins[0], GPIO.LOW)
    GPIO.output(motor_pins[1], GPIO.HIGH)
    GPIO.output(motor_pins[2], GPIO.HIGH)
    GPIO.output(motor_pins[3], GPIO.LOW)

# step 3 - low high low high
def _phase_three(motor_pins):
    GPIO.output(motor_pins[0], GPIO.LOW)
    GPIO.output(motor_pins[1], GPIO.HIGH)
    GPIO.output(motor_pins[2], GPIO.LOW)
    GPIO.output(motor_pins[3], GPIO.HIGH)

# step 4 - high low low high
def _phase_four(motor_pins):
    GPIO.output(motor_pins[0], GPIO.HIGH)
    GPIO.output(motor_pins[1], GPIO.LOW)
    GPIO.output(motor_pins[2], GPIO.LOW)
    GPIO.output(motor_pins[3], GPIO.HIGH)


def run_motors(motor_one_pins, motor_two_pins, motor_three_pins, motor_four_pins, time_sleep, revolutions):
    '''
    runs all four motors, should be used to test them
    '''
    for i in range(revolutions):
        phase_one(motor_one_pins)
        phase_one(motor_two_pins)
        phase_one(motor_three_pins)
        phase_one(motor_four_pins)
        time.sleep(time_sleep)
        
        phase_two(motor_one_pins)
        phase_two(motor_two_pins)
        phase_two(motor_three_pins)
        phase_two(motor_four_pins)
        time.sleep(time_sleep)
        
        phase_three(motor_one_pins)
        phase_three(motor_two_pins)
        phase_three(motor_three_pins)
        phase_three(motor_four_pins)
        time.sleep(time_sleep)
        
        phase_four(motor_one_pins)
        phase_four(motor_two_pins)
        phase_four(motor_three_pins)
        phase_four(motor_four_pins)
        time.sleep(time_sleep)


# functions for robot movement
def drive_north():
    # read sensor value here
    # change step times for each wheel to accomodate for it


def drive_south():
    # read sensor value here


def drive_east():
    # read sensor values here
    
def drive_west():
    # read sensor values here

def drive_diag_NE():
    # read sensor values here


def drive_diag_NW():
    # read sensor values here

def drive_diag_SW():
    # read sensor values here

def drive_diag_SE():
    # read sensor values here



# void rotateClock(){         //drive straight
# //  readSensorVal();
#   frontRightMotor->setSpeed(50);   //at a speed of 50
#   frontLeftMotor->setSpeed(50);
#   backLeftMotor->setSpeed(50);
#   backRightMotor->setSpeed(50);
#   
#   frontRightMotor->run(BACKWARD);   //at a speed of 50
#   frontLeftMotor->run(FORWARD);
#   backLeftMotor->run(FORWARD);
#   backRightMotor->run(BACKWARD);
# }
# 
# void rotateCounterClock(){         //drive straight
# //  readSensorVal();
#   frontRightMotor->setSpeed(50);   //at a speed of 50
#   frontLeftMotor->setSpeed(50);
#   backLeftMotor->setSpeed(50);
#   backRightMotor->setSpeed(50);
#   
#   frontRightMotor->run(FORWARD);   //at a speed of 50
#   frontLeftMotor->run(BACKWARD);
#   backLeftMotor->run(BACKWARD);
#   backRightMotor->run(FORWARD);
# }
# 
