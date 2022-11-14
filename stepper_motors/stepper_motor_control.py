import RPi.GPIO as GPIO
import time
import stepper_helpers


# Connect GPIO to [IN1 , IN2 , IN3 ,IN4] on Motor PCB
MOTOR_ONE_PINS = [4, 17, 27, 22]
MOTOR_TWO_PINS = [18, 23, 24, 25]
MOTOR_THREE_PINS = [21,20,16,12]
MOTOR_FOUR_PINS = [6, 13, 19, 26]


# 0.0005 is the minimum time difference i've seen
TIME_SLEEP = 0.002



# we're using the GPIO numbering system, not the board
GPIO.setmode(GPIO.BCM)






def main():
    """
    main function loop.
    """
    setup_pins(MOTOR_ONE_PINS)
    setup_pins(MOTOR_TWO_PINS)
    setup_pins(MOTOR_THREE_PINS)
    setup_pins(MOTOR_FOUR_PINS)
    
    stepper_helpers.run_motors(MOTOR_ONE_PINS, MOTOR_ONE_PINS, MOTOR_ONE_PINS, MOTOR_ONE_PINS, TIME_SLEEP, 1000):




main()
GPIO.cleanup()
