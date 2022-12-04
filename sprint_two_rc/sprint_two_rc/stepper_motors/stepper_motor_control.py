import RPi.GPIO as GPIO
import time


# Connect GPIO to [IN1 , IN2 , IN3 ,IN4] on Motor PCB
# note that this follows the gpio numbering, not the pin numbering. If you want pin numbering, change the first line of `setup_all_pins()`
MOTOR_ONE_PINS = [7, 11, 13, 15]    # back left
MOTOR_TWO_PINS = [12, 16, 18, 22]   # back right
MOTOR_THREE_PINS = [29, 31, 33, 35]  # front left
MOTOR_FOUR_PINS = [32, 36, 38, 40]  # front right




NUM_STEPS_PER_CMD = 50

# 0.0005 is the minimum time difference i've seen
# represents the sleep time (directly correlating to the wheel speed)
time_sleep = 0.004

def setup_pins(motor_pins):
    for pin in motor_pins:
        print("pin", pin)
        GPIO.setup(pin, GPIO.OUT)

def setup_all_pins():
    # we're using the GPIO numbering system, not the board
    GPIO.setmode(GPIO.BOARD)
    setup_pins(MOTOR_ONE_PINS)
    setup_pins(MOTOR_TWO_PINS)
    setup_pins(MOTOR_THREE_PINS)
    setup_pins(MOTOR_FOUR_PINS)
    print("done setting up pins")

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



# phase 1 reversed
def phase_one_reversed(motor_pins):
    GPIO.output(motor_pins[0], GPIO.LOW)
    GPIO.output(motor_pins[1], GPIO.HIGH)
    GPIO.output(motor_pins[2], GPIO.LOW)
    GPIO.output(motor_pins[3], GPIO.HIGH)

# phase 3 reversed
def phase_three_reversed(motor_pins):
    GPIO.output(motor_pins[0], GPIO.HIGH)
    GPIO.output(motor_pins[1], GPIO.LOW)
    GPIO.output(motor_pins[2], GPIO.HIGH)
    GPIO.output(motor_pins[3], GPIO.LOW)




# functions for robot movement

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


def reverse_motors(motor_one_pins, motor_two_pins, motor_three_pins, motor_four_pins, time_sleep, revolutions):
    '''
    runs all four motors in reverse. Should be used to test them
    '''
    for i in range(revolutions):
        phase_one_reversed(motor_one_pins)
        phase_one_reversed(motor_two_pins)
        phase_one_reversed(motor_three_pins)
        phase_one_reversed(motor_four_pins)
        time.sleep(time_sleep)
        
        phase_two(motor_one_pins)
        phase_two(motor_two_pins)
        phase_two(motor_three_pins)
        phase_two(motor_four_pins)
        time.sleep(time_sleep)
        
        phase_three_reversed(motor_one_pins)
        phase_three_reversed(motor_two_pins)
        phase_three_reversed(motor_three_pins)
        phase_three_reversed(motor_four_pins)
        time.sleep(time_sleep)
        
        phase_four(motor_one_pins)
        phase_four(motor_two_pins)
        phase_four(motor_three_pins)
        phase_four(motor_four_pins)
        time.sleep(time_sleep)



def drive_north():
    '''
    runs the motors in a north direction for one step. should be called in a loop
    '''
    # TODO: read sensor value here

    # all wheels at the same speed, and moves forward
    for _ in range(NUM_STEPS_PER_CMD):
        phase_one_reversed(MOTOR_ONE_PINS)
        phase_one(MOTOR_TWO_PINS)
        phase_one_reversed(MOTOR_THREE_PINS)
        phase_one(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)
    
        phase_two(MOTOR_ONE_PINS)
        phase_two(MOTOR_TWO_PINS)
        phase_two(MOTOR_THREE_PINS)
        phase_two(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)
    
        phase_three_reversed(MOTOR_ONE_PINS)
        phase_three(MOTOR_TWO_PINS)
        phase_three_reversed(MOTOR_THREE_PINS)
        phase_three(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)
    
        phase_four(MOTOR_ONE_PINS)
        phase_four(MOTOR_TWO_PINS)
        phase_four(MOTOR_THREE_PINS)
        phase_four(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)


def drive_south():
    # read sensor value here
    # all wheels at the same speed, and moves backwards
    for _ in range(NUM_STEPS_PER_CMD):
        phase_one(MOTOR_ONE_PINS)
        phase_one_reversed(MOTOR_TWO_PINS)
        phase_one(MOTOR_THREE_PINS)
        phase_one_reversed(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)
        
        phase_two(MOTOR_ONE_PINS)
        phase_two(MOTOR_TWO_PINS)
        phase_two(MOTOR_THREE_PINS)
        phase_two(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)
        
        phase_three(MOTOR_ONE_PINS)
        phase_three_reversed(MOTOR_TWO_PINS)
        phase_three(MOTOR_THREE_PINS)
        phase_three_reversed(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)
        
        phase_four(MOTOR_ONE_PINS)
        phase_four(MOTOR_TWO_PINS)
        phase_four(MOTOR_THREE_PINS)
        phase_four(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)


def drive_east():
    # read sensor values here
    # all wheels at the same speed, front right/back left go backwards, and other two go forwards
    for _ in range(NUM_STEPS_PER_CMD):
        phase_one(MOTOR_ONE_PINS)
        phase_one_reversed(MOTOR_TWO_PINS)
        phase_one_reversed(MOTOR_THREE_PINS)
        phase_one(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)
        
        phase_two(MOTOR_ONE_PINS)
        phase_two(MOTOR_TWO_PINS)
        phase_two(MOTOR_THREE_PINS)
        phase_two(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)
        
        phase_three(MOTOR_ONE_PINS)
        phase_three_reversed(MOTOR_TWO_PINS)
        phase_three_reversed(MOTOR_THREE_PINS)
        phase_three(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)
        
        phase_four(MOTOR_ONE_PINS)
        phase_four(MOTOR_TWO_PINS)
        phase_four(MOTOR_THREE_PINS)
        phase_four(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)



def drive_west():
    # read sensor values here
    # all wheels at the same speed, front right/back left go forwards, other two go backwards
    for _ in range(NUM_STEPS_PER_CMD):
        phase_one(MOTOR_ONE_PINS)
        phase_one(MOTOR_TWO_PINS)
        phase_one_reversed(MOTOR_THREE_PINS)
        phase_one_reversed(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)
        
        phase_two(MOTOR_ONE_PINS)
        phase_two(MOTOR_TWO_PINS)
        phase_two(MOTOR_THREE_PINS)
        phase_two(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)
        
        phase_three(MOTOR_ONE_PINS)
        phase_three(MOTOR_TWO_PINS)
        phase_three_reversed(MOTOR_THREE_PINS)
        phase_three_reversed(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)
        
        phase_four(MOTOR_ONE_PINS)
        phase_four(MOTOR_TWO_PINS)
        phase_four(MOTOR_THREE_PINS)
        phase_four(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)


def drive_diag_NE():
    # read sensor values here
    # front right/back left don't move, front left/back right move forward
    for _ in range(NUM_STEPS_PER_CMD):
        phase_one_reversed(MOTOR_ONE_PINS)
        phase_one(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)
        
        phase_two(MOTOR_ONE_PINS)
        phase_two(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)
        
        phase_three_reversed(MOTOR_ONE_PINS)
        phase_three(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)
        
        phase_four(MOTOR_ONE_PINS)
        phase_four(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)


def drive_diag_NW():
    # read sensor values here
    # front right/back left move forwards, front left/back right don't move
    for _ in range(NUM_STEPS_PER_CMD):
        phase_one(MOTOR_TWO_PINS)
        phase_one_reversed(MOTOR_THREE_PINS)
        time.sleep(time_sleep)
        
        phase_two(MOTOR_TWO_PINS)
        phase_two(MOTOR_THREE_PINS)
        time.sleep(time_sleep)
        
        phase_three(MOTOR_TWO_PINS)
        phase_three_reversed(MOTOR_THREE_PINS)
        time.sleep(time_sleep)
        
        phase_four(MOTOR_TWO_PINS)
        phase_four(MOTOR_THREE_PINS)
        time.sleep(time_sleep)


def drive_diag_SW():
    # read sensor values here
    # front right and back left motors don't move. front left and back right move at the same speeds and Bacjwards.
    for _ in range(NUM_STEPS_PER_CMD):
        phase_one(MOTOR_ONE_PINS)
        phase_one_reversed(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)
        
        phase_two(MOTOR_ONE_PINS)
        phase_two(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)
        
        phase_three(MOTOR_ONE_PINS)
        phase_three_reversed(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)
        
        phase_four(MOTOR_ONE_PINS)
        phase_four(MOTOR_FOUR_PINS)
        time.sleep(time_sleep)


def drive_diag_SE():
    # read sensor values here
    # front right and back left motors move at speeds and bakcwards. Other two don't move
    for _ in range(NUM_STEPS_PER_CMD):
        phase_one_reversed(MOTOR_TWO_PINS)
        phase_one(MOTOR_THREE_PINS)
        time.sleep(time_sleep)
        
        phase_two(MOTOR_TWO_PINS)
        phase_two(MOTOR_THREE_PINS)
        time.sleep(time_sleep)
        
        phase_three_reversed(MOTOR_TWO_PINS)
        phase_three(MOTOR_THREE_PINS)
        time.sleep(time_sleep)
        
        phase_four(MOTOR_TWO_PINS)
        phase_four(MOTOR_THREE_PINS)
        time.sleep(time_sleep)


def rotate_clock():
    # read sensor value
    # front left and back left move forward. other two move backwards
    phase_one(MOTOR_ONE_PINS)
    phase_one_reversed(MOTOR_TWO_PINS)
    phase_one(MOTOR_THREE_PINS)
    phase_one_reversed(MOTOR_FOUR_PINS)
    time.sleep(time_sleep)
    
    phase_two(MOTOR_ONE_PINS)
    phase_two(MOTOR_TWO_PINS)
    phase_two(MOTOR_THREE_PINS)
    phase_two(MOTOR_FOUR_PINS)
    time.sleep(time_sleep)
    
    phase_three(MOTOR_ONE_PINS)
    phase_three_reversed(MOTOR_TWO_PINS)
    phase_three(MOTOR_THREE_PINS)
    phase_three_reversed(MOTOR_FOUR_PINS)
    time.sleep(time_sleep)
    
    phase_four(MOTOR_ONE_PINS)
    phase_four(MOTOR_TWO_PINS)
    phase_four(MOTOR_THREE_PINS)
    phase_four(MOTOR_FOUR_PINS)
    time.sleep(time_sleep)


def rotate_counter_clock():
    # front left and back left move backward. other two move forward
    phase_one_reversed(MOTOR_ONE_PINS)
    phase_one(MOTOR_TWO_PINS)
    phase_one_reversed(MOTOR_THREE_PINS)
    phase_one(MOTOR_FOUR_PINS)
    time.sleep(time_sleep)
    
    phase_two(MOTOR_ONE_PINS)
    phase_two(MOTOR_TWO_PINS)
    phase_two(MOTOR_THREE_PINS)
    phase_two(MOTOR_FOUR_PINS)
    time.sleep(time_sleep)
    
    phase_three_reversed(MOTOR_ONE_PINS)
    phase_three(MOTOR_TWO_PINS)
    phase_three_reversed(MOTOR_THREE_PINS)
    phase_three(MOTOR_FOUR_PINS)
    time.sleep(time_sleep)
    
    phase_four(MOTOR_ONE_PINS)
    phase_four(MOTOR_TWO_PINS)
    phase_four(MOTOR_THREE_PINS)
    phase_four(MOTOR_FOUR_PINS)
    time.sleep(time_sleep)





# print("running")
setup_all_pins()
# run_motors(MOTOR_ONE_PINS, MOTOR_TWO_PINS, MOTOR_THREE_PINS, MOTOR_FOUR_PINS, 0.002, 500)
drive_north()
print("done")
# time.sleep(1)
# print("reversing")
# reverse_motors(MOTOR_ONE_PINS, MOTOR_TWO_PINS, MOTOR_THREE_PINS, MOTOR_FOUR_PINS, 0.002, 500)

# for i in range(10000):
#     run_motors(MOTOR_ONE_PINS, MOTOR_TWO_PINS, MOTOR_THREE_PINS, MOTOR_FOUR_PINS, TIME_SLEEP, 1000)
#     
#     if i < 500:
#         stepper_helpers.drive_north()
#     else if i < 1500:
#         stepper_helpers.drive_south()



GPIO.cleanup()





# TODO: make reverse function work. make half stepping
