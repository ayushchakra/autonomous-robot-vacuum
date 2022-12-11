#include <Adafruit_MotorShield.h>                     //include motor shield library

Adafruit_MotorShield AFMS = Adafruit_MotorShield();   //create Adafruit_MotorShield object
Adafruit_DCMotor *frontLeftMotor = AFMS.getMotor(2);      //assign the right motor to port M1
Adafruit_DCMotor *frontRightMotor = AFMS.getMotor(1);       //assign the left motor to port M3
Adafruit_DCMotor *backLeftMotor = AFMS.getMotor(3);      //assign the right motor to port M1
Adafruit_DCMotor *backRightMotor = AFMS.getMotor(4);       //assign the left motor to port M3

int currDriveMode = 0;

void setup() {
  Serial.flush();                                     //reset serial monitor
  Serial.begin(9600);

  AFMS.begin();                                       //begin the Adafruit_MotorShield object
}

void loop() {
  if (Serial.available() > 0) {
//    currDriveMode = Serial.parseInt();
//    Serial.write(currDriveMode);
    String a = Serial.readString();
    currDriveMode = a.toInt();
//    Serial.println(currDriveMode);
  }
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
    driveDiagNE();
  }
  else if(currDriveMode==6) {
    driveDiagNW();
  }
  else if(currDriveMode==7) {
    driveDiagSE();
  }
  else if(currDriveMode==8) {
    driveDiagSW();
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
//  switch (currDriveMode) {
//    case 1:
//      driveNorth();
//      Serial.println(1);
//    case 2:
//      driveSouth();
//      Serial.println(2);
//    case 3:
//      driveEast();
//      Serial.println(3);
//    case 4:
//      driveWest();
//    case 5:
//      driveDiagNE();
//    case 6:
//      driveDiagNW();
//    case 7:
//      driveDiagSE();
//    case 8:
//      driveDiagSW();
//    case 9:
//      rotateClock();
//    case 10:
//      rotateCounterClock();
//  }
}

void driveNorth(){         //drive straight
//  readSensorVal();
  frontRightMotor->setSpeed(50);   //at a speed of 50
  frontLeftMotor->setSpeed(50);
  backLeftMotor->setSpeed(50);
  backRightMotor->setSpeed(50);
  
  frontRightMotor->run(FORWARD);   //at a speed of 50
  frontLeftMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);
}

void driveSouth(){         //drive straight
//  readSensorVal();
  frontRightMotor->setSpeed(50);   //at a speed of 50
  frontLeftMotor->setSpeed(50);
  backLeftMotor->setSpeed(50);
  backRightMotor->setSpeed(50);
  
  frontRightMotor->run(BACKWARD);   //at a speed of 50
  frontLeftMotor->run(BACKWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(BACKWARD);
}

void driveEast(){         //drive straight
//  readSensorVal();
  frontRightMotor->setSpeed(50);   //at a speed of 50
  frontLeftMotor->setSpeed(50);
  backLeftMotor->setSpeed(50);
  backRightMotor->setSpeed(50);
  
  frontRightMotor->run(BACKWARD);   //at a speed of 50
  frontLeftMotor->run(FORWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(FORWARD);
}

void driveWest(){         //drive straight
//  readSensorVal();
  frontRightMotor->setSpeed(50);   //at a speed of 50
  frontLeftMotor->setSpeed(50);
  backLeftMotor->setSpeed(50);
  backRightMotor->setSpeed(50);
  
  frontRightMotor->run(FORWARD);   //at a speed of 50
  frontLeftMotor->run(BACKWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(BACKWARD);
}

void driveDiagNE(){         //drive straight
//  readSensorVal();
  frontRightMotor->setSpeed(0);   //at a speed of 50
  frontLeftMotor->setSpeed(50);
  backLeftMotor->setSpeed(0);
  backRightMotor->setSpeed(50);
  
  frontRightMotor->run(FORWARD);   //at a speed of 50
  frontLeftMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);
}

void driveDiagNW(){         //drive straight
//  readSensorVal();
  frontRightMotor->setSpeed(50);   //at a speed of 50
  frontLeftMotor->setSpeed(0);
  backLeftMotor->setSpeed(50);
  backRightMotor->setSpeed(0);
  
  frontRightMotor->run(FORWARD);   //at a speed of 50
  frontLeftMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);
}

void driveDiagSW(){         //drive straight
//  readSensorVal();
  frontRightMotor->setSpeed(0);   //at a speed of 50
  frontLeftMotor->setSpeed(50);
  backLeftMotor->setSpeed(0);
  backRightMotor->setSpeed(50);
  
  frontRightMotor->run(RELEASE);   //at a speed of 50
  frontLeftMotor->run(BACKWARD);
  backLeftMotor->run(RELEASE);
  backRightMotor->run(BACKWARD);
}

void driveDiagSE(){         //drive straight
//  readSensorVal();
  frontRightMotor->setSpeed(50);   //at a speed of 50
  frontLeftMotor->setSpeed(0);
  backLeftMotor->setSpeed(50);
  backRightMotor->setSpeed(0);
  
  frontRightMotor->run(BACKWARD);   //at a speed of 50
  frontLeftMotor->run(RELEASE);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(RELEASE);
}

void rotateClock(){         //drive straight
//  readSensorVal();
  frontRightMotor->setSpeed(50);   //at a speed of 50
  frontLeftMotor->setSpeed(50);
  backLeftMotor->setSpeed(50);
  backRightMotor->setSpeed(50);
  
  frontRightMotor->run(BACKWARD);   //at a speed of 50
  frontLeftMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(BACKWARD);
}

void rotateCounterClock(){         //drive straight
//  readSensorVal();
  frontRightMotor->setSpeed(50);   //at a speed of 50
  frontLeftMotor->setSpeed(50);
  backLeftMotor->setSpeed(50);
  backRightMotor->setSpeed(50);
  
  frontRightMotor->run(FORWARD);   //at a speed of 50
  frontLeftMotor->run(BACKWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(FORWARD);
}

void tempStop() {
    frontRightMotor->setSpeed(0);   //at a speed of 50
  frontLeftMotor->setSpeed(0);
  backLeftMotor->setSpeed(0);
  backRightMotor->setSpeed(0);
  
  frontRightMotor->run(FORWARD);   //at a speed of 50
  frontLeftMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);
}
