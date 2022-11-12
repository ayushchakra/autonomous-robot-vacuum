import RPi.GPIO as GPIO
import time
from RpiMotorLib import RpiMotorLib


mymotortest = RpiMotorLib.BYJMotor("MyMotorOne", "Nema")

# Connect GPIO to [IN1 , IN2 , IN3 ,IN4] on Motor PCB
GpioPins = [7, 11, 13, 15]

# 
MOTOR_ONE_PINS = [4, 17, 27, 22]  # [7, 0, 2, 3]
MOTOR_TWO_PINS = [6, 13, 19, 26]
MOTOR_THREE_PINS = [21,20,16,12]  # [12,16,20,21]
MOTOR_FOUR_PINS = [35, 37, 38, 40]




# step 1 - high low high low
def phase_one(motor_array):
    GPIO.output(motor_array[0], GPIO.HIGH)
    GPIO.output(motor_array[1], GPIO.LOW)
    GPIO.output(motor_array[2], GPIO.HIGH)
    GPIO.output(motor_array[3], GPIO.LOW)

# step 2 - low high high low
def phase_two(motor_array):
    GPIO.output(motor_array[0], GPIO.LOW)
    GPIO.output(motor_array[1], GPIO.HIGH)
    GPIO.output(motor_array[2], GPIO.HIGH)
    GPIO.output(motor_array[3], GPIO.LOW)

# step 3 - low high low high
def phase_three(motor_array):
    GPIO.output(motor_array[0], GPIO.LOW)
    GPIO.output(motor_array[1], GPIO.HIGH)
    GPIO.output(motor_array[2], GPIO.LOW)
    GPIO.output(motor_array[3], GPIO.HIGH)

# step 4 - high low low high
def phase_four(motor_array):
    GPIO.output(motor_array[0], GPIO.HIGH)
    GPIO.output(motor_array[1], GPIO.LOW)
    GPIO.output(motor_array[2], GPIO.LOW)
    GPIO.output(motor_array[3], GPIO.HIGH)


def main():
    """
    main function loop.
    """
    for pin in MOTOR_ONE_PINS:
        GPIO.setup(pin, GPIO.OUT)
    
    for pin in MOTOR_THREE_PINS:
        GPIO.setup(pin, GPIO.OUT)
    
    for i in range(100):
        phase_one(MOTOR_ONE_PINS)
        phase_one(MOTOR_THREE_PINS)
        time.sleep(0.005)
        
        phase_two(MOTOR_ONE_PINS)
        phase_two(MOTOR_THREE_PINS)
        time.sleep(0.005)
        
        phase_three(MOTOR_ONE_PINS)
        phase_three(MOTOR_THREE_PINS)
        time.sleep(0.005)
        
        phase_four(MOTOR_ONE_PINS)
        phase_four(MOTOR_THREE_PINS)
        time.sleep(0.005)
    
    
    
    # step 2
    
    # Arguments  for motor run function
    # (GPIOPins, stepdelay, steps, counterclockwise, verbose, steptype, initdelay)
#     input("Press <Enter> to test motor 1")
#     mymotortest.motor_run(MOTOR_ONE_PINS,.5, 50, False, True,"full" ,1 )
#     time.sleep(1)
#     
#     input("Press <Enter> to test motor 2")
#     mymotortest.motor_run(MOTOR_TWO_PINS,.2, 10, False, True,"full", 1)
#     time.sleep(1) 
#     input("Press <Enter> to test motor 3")
#     mymotortest.motor_run(MOTOR_THREE_PINS,.05, 25, True, True,"half", 1)
#     time.sleep(1)
#     input("Press <Enter> to test motor 4")
#     mymotortest.motor_run(MOTOR_FOUR_PINS,.02,25,False,False,"half", 3)


main()
GPIO.cleanup()
