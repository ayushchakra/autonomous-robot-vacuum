import RPi.GPIO as GPIO
import time
from stepper_helpers import setup_pins, phase_one, phase_two, phase_three, phase_four


# Connect GPIO to [IN1 , IN2 , IN3 ,IN4] on Motor PCB
MOTOR_ONE_PINS = [4, 17, 27, 22]
MOTOR_TWO_PINS = [18, 23, 24, 25]
MOTOR_THREE_PINS = [21,20,16,12]
MOTOR_FOUR_PINS = [6, 13, 19, 26]


# 0.0005 is the minimum time difference i've seen
TIME_SLEEP = 0.002



# we're using the GPIO numbering system, not the board
GPIO.setmode(GPIO.BCM)


setup_pins(MOTOR_ONE_PINS)
setup_pins(MOTOR_TWO_PINS)
setup_pins(MOTOR_THREE_PINS)
setup_pins(MOTOR_FOUR_PINS)



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









# this is the function that shuold be called in the lidar sensing code. Or, run the loop in here and constantly read inputs from the lidar
run_motors(MOTOR_ONE_PINS, MOTOR_ONE_PINS, MOTOR_ONE_PINS, MOTOR_ONE_PINS, TIME_SLEEP, 1000)


GPIO.cleanup()
