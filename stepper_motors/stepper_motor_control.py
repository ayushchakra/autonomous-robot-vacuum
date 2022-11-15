import RPi.GPIO as GPIO
import time


# Connect GPIO to [IN1 , IN2 , IN3 ,IN4] on Motor PCB
MOTOR_ONE_PINS = [4, 17, 27, 22]
MOTOR_TWO_PINS = [18, 23, 24, 25]
MOTOR_THREE_PINS = [21,20,16,12]
MOTOR_FOUR_PINS = [6, 13, 19, 26]


# 0.0005 is the minimum time difference i've seen
# represents the sleep time (directly correlating to the wheel speed)
time_sleep_1 = 0.002
time_sleep_2 = 0.002
time_sleep_3 = 0.002
time_sleep_4 = 0.002






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
def drive_north():
    # read sensor value here
    # all wheels at the same speed, and moves forward
    time_sleep_1 = 0.002
    time_sleep_2 = 0.002
    time_sleep_3 = 0.002
    time_sleep_4 = 0.002


def drive_south():
    # read sensor value here
    # all wheels at the same speed, and moves backwards
    time_sleep_1 = 0.002
    time_sleep_2 = 0.002
    time_sleep_3 = 0.002
    time_sleep_4 = 0.002


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
    runs all four motors in reverse
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






# we're using the GPIO numbering system, not the board
GPIO.setmode(GPIO.BCM)


setup_pins(MOTOR_ONE_PINS)
setup_pins(MOTOR_TWO_PINS)
setup_pins(MOTOR_THREE_PINS)
setup_pins(MOTOR_FOUR_PINS)


print("running")
run_motors(MOTOR_ONE_PINS, MOTOR_TWO_PINS, MOTOR_THREE_PINS, MOTOR_FOUR_PINS, 0.002, 500)
print("reversing")
reverse_motors(MOTOR_ONE_PINS, MOTOR_TWO_PINS, MOTOR_THREE_PINS, MOTOR_FOUR_PINS, 0.002, 500)

# for i in range(10000):
#     run_motors(MOTOR_ONE_PINS, MOTOR_TWO_PINS, MOTOR_THREE_PINS, MOTOR_FOUR_PINS, TIME_SLEEP, 1000)
#     
#     if i < 500:
#         stepper_helpers.drive_north()
#     else if i < 1500:
#         stepper_helpers.drive_south()




GPIO.cleanup()





# TODO: make reverse function work. make half stepping