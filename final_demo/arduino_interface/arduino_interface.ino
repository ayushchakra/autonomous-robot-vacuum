#include <Adafruit_MotorShield.h>                       //include motor shield library

Adafruit_MotorShield AFMS = Adafruit_MotorShield();     //create Adafruit_MotorShield object
Adafruit_DCMotor *frontLeftMotor = AFMS.getMotor(2);    //assign the right motor to port M1
Adafruit_DCMotor *frontRightMotor = AFMS.getMotor(1);   //assign the left motor to port M3
Adafruit_DCMotor *backLeftMotor = AFMS.getMotor(3);     //assign the right motor to port M1
Adafruit_DCMotor *backRightMotor = AFMS.getMotor(4);    //assign the left motor to port M3

int currDriveMode = 0;                                  //int that represents the current drive state (direction)
int DRIVE_SPEED = 50;                                   //int that represents the drive speed of the robot

void setup() {
  Serial.flush();                                       //reset serial monitor
  Serial.begin(9600);

  AFMS.begin();                                         //begin the Adafruit_MotorShield object
}

void loop() {
  //read the serial monitor to check to see if an update to the drive state has been requested
  if (Serial.available() > 0) {
    //if there is a new updated value, it is read and processed as an Integer
    String a = Serial.readString();
    currDriveMode = a.toInt();
  }

  //based on the current drive mode, the appropriate drive command is called
  if(currDriveMode==1) {
    driveNorth();
  }
  else if(currDriveMode==2) {
    driveSouth();
  }
  else if(currDriveMode==3) {
    driveEast();
  }
  else if(currDriveMode==4) {
    driveWest();
  }
  else if(currDriveMode==5) {
    driveNorthEast();
  }
  else if(currDriveMode==6) {
    driveNorthWest();
  }
  else if(currDriveMode==7) {
    driveSouthEast();
  }
  else if(currDriveMode==8) {
    driveSouthWest();
  }
  else if(currDriveMode==9) {
    rotateClock();
  }
  else if(currDriveMode==10) {
    rotateCounterClock();
  }
  else if(currDriveMode==11) {
    tempStop();
  }
}

void driveNorth(){
  //drive straight

  //set the drive speeds
  frontRightMotor->setSpeed(DRIVE_SPEED);
  frontLeftMotor->setSpeed(DRIVE_SPEED);
  backLeftMotor->setSpeed(DRIVE_SPEED);
  backRightMotor->setSpeed(DRIVE_SPEED);
  
  //all motors should be going forward
  frontRightMotor->run(FORWARD);
  frontLeftMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);
}

void driveSouth(){
  //drive backwards

  //set the drive speeds
  frontRightMotor->setSpeed(DRIVE_SPEED);
  frontLeftMotor->setSpeed(DRIVE_SPEED);
  backLeftMotor->setSpeed(DRIVE_SPEED);
  backRightMotor->setSpeed(DRIVE_SPEED);
  
  //all motors should be going
  frontRightMotor->run(BACKWARD);
  frontLeftMotor->run(BACKWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(BACKWARD);
}

void driveEast(){
  //drive to the right

  //set the drive speeds
  frontLeftMotor->setSpeed(DRIVE_SPEED);
  backLeftMotor->setSpeed(DRIVE_SPEED);
  backRightMotor->setSpeed(DRIVE_SPEED);

  //diagonal motars are the same direction and front right should be going
  //backwards  
  frontRightMotor->run(BACKWARD);
  frontLeftMotor->run(FORWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(FORWARD);
}

void driveWest(){
  //drive to the left

  //set the drive speeds
  frontRightMotor->setSpeed(DRIVE_SPEED);
  frontLeftMotor->setSpeed(DRIVE_SPEED);
  backLeftMotor->setSpeed(DRIVE_SPEED);
  backRightMotor->setSpeed(DRIVE_SPEED);
  
  //motors should be in the opposite direction of driveEast()
  frontRightMotor->run(FORWARD);
  frontLeftMotor->run(BACKWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(BACKWARD);
}

void driveNorthEast(){
  //drive forward and to the right

  //set the drive speeds (front right and back left are stationary)
  frontRightMotor->setSpeed(0);
  frontLeftMotor->setSpeed(DRIVE_SPEED);
  backLeftMotor->setSpeed(0);
  backRightMotor->setSpeed(DRIVE_SPEED);
  

  //back right and front left are set to forward
  frontLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);
}

void driveNorthWest(){
  //drive forward and to the left

  //set the drive speeds (front left and back right are stationary)
  frontRightMotor->setSpeed(DRIVE_SPEED);
  frontLeftMotor->setSpeed(0);
  backLeftMotor->setSpeed(DRIVE_SPEED);
  backRightMotor->setSpeed(0);
  

  //back left and front right are set to forward
  frontRightMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
}

void driveSouthWest(){
  //drive backward and to the left

  //set the drive speeds (front right and back left are stationary)
  frontRightMotor->setSpeed(0);
  frontLeftMotor->setSpeed(DRIVE_SPEED);
  backLeftMotor->setSpeed(0);
  backRightMotor->setSpeed(DRIVE_SPEED);
  
  //opposite directions as driveNorthEast()
  frontLeftMotor->run(BACKWARD);
  backRightMotor->run(BACKWARD);
}

void driveSouthEast(){
  //drive backward and to the right

  //set the drive speeds (back right and front left are stationary)
  frontRightMotor->setSpeed(DRIVE_SPEED);
  frontLeftMotor->setSpeed(0);
  backLeftMotor->setSpeed(DRIVE_SPEED);
  backRightMotor->setSpeed(0);

  //opposite directions as driveNorthWest()  
  frontRightMotor->run(BACKWARD);
  backLeftMotor->run(BACKWARD);
}

void rotateClock(){
  //rotate clockwise

  //set the drive speeds
  frontRightMotor->setSpeed(DRIVE_SPEED);
  frontLeftMotor->setSpeed(DRIVE_SPEED);
  backLeftMotor->setSpeed(DRIVE_SPEED);
  backRightMotor->setSpeed(DRIVE_SPEED);
  
  //right wheels backward and left wheels forward
  frontRightMotor->run(BACKWARD);
  frontLeftMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(BACKWARD);
}

void rotateCounterClock(){
  //rotate counterclockwise

  //set the drive speeds
  frontRightMotor->setSpeed(DRIVE_SPEED);
  frontLeftMotor->setSpeed(DRIVE_SPEED);
  backLeftMotor->setSpeed(DRIVE_SPEED);
  backRightMotor->setSpeed(DRIVE_SPEED);

  //opposite direction as rotateClock()
  frontRightMotor->run(FORWARD);
  frontLeftMotor->run(BACKWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(FORWARD);
}

void tempStop() {
  //stop motor motion temporarily

  //set each motor to 0
  frontRightMotor->setSpeed(0);
  frontLeftMotor->setSpeed(0);
  backLeftMotor->setSpeed(0);
  backRightMotor->setSpeed(0);
}
