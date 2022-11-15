'''
Helper functions for the stepper motors
'''

import RPi.GPIO as GPIO
import time



def setup_pins(motor_pins):
    for pin in motor_pins:
        GPIO.setup(pin, GPIO.OUT)


# step 1 - high low high low
def phase_one(motor_pins):
    GPIO.output(motor_pins[0], GPIO.HIGH)
    GPIO.output(motor_pins[1], GPIO.LOW)
    GPIO.output(motor_pins[2], GPIO.HIGH)
    GPIO.output(motor_pins[3], GPIO.LOW)

# step 2 - low high high low
def phase_two(motor_pins):
    GPIO.output(motor_pins[0], GPIO.LOW)
    GPIO.output(motor_pins[1], GPIO.HIGH)
    GPIO.output(motor_pins[2], GPIO.HIGH)
    GPIO.output(motor_pins[3], GPIO.LOW)

# step 3 - low high low high
def phase_three(motor_pins):
    GPIO.output(motor_pins[0], GPIO.LOW)
    GPIO.output(motor_pins[1], GPIO.HIGH)
    GPIO.output(motor_pins[2], GPIO.LOW)
    GPIO.output(motor_pins[3], GPIO.HIGH)

# step 4 - high low low high
def phase_four(motor_pins):
    GPIO.output(motor_pins[0], GPIO.HIGH)
    GPIO.output(motor_pins[1], GPIO.LOW)
    GPIO.output(motor_pins[2], GPIO.LOW)
    GPIO.output(motor_pins[3], GPIO.HIGH)



# functions for robot movement
def drive_north():
    # read sensor value here
    # all wheels at the same speed, and moves forward
    pass


def drive_south():
    # read sensor value here
    # all wheels at the same speed, and moves backwards
    pass


def drive_east():
    # read sensor values here
    # all wheels at the same speed, front right/back left go backwards, and other two go forwards
    pass

    
def drive_west():
    # read sensor values here
    # all wheels at the same speed, front right/back left go forwards, other two go backwards
    pass


def drive_diag_NE():
    # read sensor values here
    # all wheels move forwards, front right/back left don't move, front left/back right move at same speeds
    pass


def drive_diag_NW():
    # read sensor values here
    # all wheels move forwards, front right/back left move at same speeds, front left/back right don't move
    pass


def drive_diag_SW():
    # read sensor values here
    # front right and back left motors don't move. front left and back right move at the same speeds and Bacjwards.
    pass


def drive_diag_SE():
    # read sensor values here
    # front right and back left motors move at speeds and bakcwards. Other two don't move
    pass




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
