import RPi.GPIO as gpio
import time

stepPin = 38
gpio.setmode(gpio.BOARD)
gpio.setup(stepPin, gpio.OUT)

for _ in range(200):
    gpio.output(stepPin, gpio.HIGH)
    time.sleep(.005)
    gpio.output(stepPin, gpio.LOW)
    time.sleep(.005)
