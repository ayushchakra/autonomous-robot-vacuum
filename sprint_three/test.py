import serial

arduino_linux=serial.Serial("/dev/ttyACM0",9600)
while True:
    print('write:',str.encode('90')) #print the sent value through the serial to check it with the readed value
    arduino_linux.write(str.encode('9'))#send the following value
