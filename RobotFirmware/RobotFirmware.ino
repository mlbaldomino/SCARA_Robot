/**
 * Martha Linares - Arduino based SCARA Robot
 * base on Dejan Code, www.HowToMechatronics.com
 */

#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

#include "StepperMotorDriver.h"

#define MAX_STEPPER_SPEED 4000
#define STEPPER_ACCELERATION 2000

#define LIMIT_SWITCH_PIN_1 11
#define LIMIT_SWITCH_PIN_2 10
#define LIMIT_SWITCH_PIN_3 9
#define LIMIT_SWITCH_PIN_4 A3

// Define the stepper motors and the pins the will use
// (Type:driver, STEP, DIR)
AccelStepper stepper1(AccelStepper::DRIVER,  2,  5);
AccelStepper stepper2(AccelStepper::DRIVER,  3,  6);
AccelStepper stepper3(AccelStepper::DRIVER,  4,  7);
AccelStepper stepper4(AccelStepper::DRIVER, 12, 13);

// Create servo object to control a servo
Servo gripperServo;

double x = 10.0;
double y = 10.0;
double L1 = 228.0;  // L1 = 228.0 mm
double L2 = 136.5;  // L2 = 136.5 mm
double theta1, theta2, phi, z;

int stepper1Position, stepper2Position, stepper3Position, stepper4Position;

const float theta1AngleToSteps = 44.444444;
const float theta2AngleToSteps = 35.555555;
const float phiAngleToSteps = 10.0;
const float zDistanceToSteps = 100.0;

byte inputValue[5] = { 0 };
int k = 0;

String content = "";
/*
 * 0 - save status
 * 1 - run status
 * 1 - j1 slider
 * 3 - j2 slider
 * 4 - j3 slider
 * 5 - z slider (heigth)
 * 6 - grip value
 * 7 - speed slider
 * 8 - acceleration slider
 * 9
 */
int data[10] = { 0 };

int theta1Array[100] = { 0 };
int theta2Array[100] = { 0 };
int phiArray[100] = { 0 };
int zArray[100] = { 0 };
int gripperArray[100] = { 0 };

int positionsCounter = 0;

void setup() {
  Serial.begin(115200);

  // Set internal pullups for input limit switch
  pinMode(LIMIT_SWITCH_PIN_1, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN_2, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN_3, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN_4, INPUT_PULLUP);

  // Set stepper motors max speed
  stepper1.setMaxSpeed(MAX_STEPPER_SPEED);
  stepper2.setMaxSpeed(MAX_STEPPER_SPEED);
  stepper3.setMaxSpeed(MAX_STEPPER_SPEED);
  stepper4.setMaxSpeed(MAX_STEPPER_SPEED);

  // Set stepper motors acceleration
  stepper1.setAcceleration(STEPPER_ACCELERATION);
  stepper2.setAcceleration(STEPPER_ACCELERATION);
  stepper3.setAcceleration(STEPPER_ACCELERATION);
  stepper4.setAcceleration(STEPPER_ACCELERATION);

  // Configure gripper servo (pin, min, max)
  gripperServo.attach(A0, 600, 2500);

  // Set initial servo value - open gripper
  data[6] = 180;
  gripperServo.write(data[6]);

  delay(1000);

  data[5] = 100;
  setHome();
}

void loop() {

  if (Serial.available()) {
    content = Serial.readString();  // Read the incomding data from Processing
    // Extract the data from the string and put into separate integer variables (data[] array)
    for (int i = 0; i < 10; i++) {
      int index = content.indexOf(",");                     // locate the first ","
      data[i] = atol(content.substring(0, index).c_str());  //Extract the number from start to the ","
      content = content.substring(index + 1);               //Remove the number from the string
    }
    /*
     data[0] - SAVE button status
     data[1] - RUN button status
     data[2] - Joint 1 angle
     data[3] - Joint 2 angle
     data[4] - Joint 3 angle
     data[5] - Z position
     data[6] - Gripper value
     data[7] - Speed value
     data[8] - Acceleration value
    */
    // If SAVE button is pressed, store the data into the appropriate arrays
    if (data[0] == 1) {
      theta1Array[positionsCounter] = data[2] * theta1AngleToSteps;  //store the values in steps = angles * angleToSteps variable
      theta2Array[positionsCounter] = data[3] * theta2AngleToSteps;
      phiArray[positionsCounter] = data[4] * phiAngleToSteps;
      zArray[positionsCounter] = data[5] * zDistanceToSteps;
      gripperArray[positionsCounter] = data[6];
      positionsCounter++;
    }
    // clear data
    if (data[0] == 2) {
      // Clear the array data to 0
      memset(theta1Array, 0, sizeof(theta1Array));
      memset(theta2Array, 0, sizeof(theta2Array));
      memset(phiArray, 0, sizeof(phiArray));
      memset(zArray, 0, sizeof(zArray));
      memset(gripperArray, 0, sizeof(gripperArray));
      positionsCounter = 0;
    }
  }
  // If RUN button is pressed
  while (data[1] == 1) {
    stepper1.setSpeed(data[7]);
    stepper2.setSpeed(data[7]);
    stepper3.setSpeed(data[7]);
    stepper4.setSpeed(data[7]);
    stepper1.setAcceleration(data[8]);
    stepper2.setAcceleration(data[8]);
    stepper3.setAcceleration(data[8]);
    stepper4.setAcceleration(data[8]);

    // execute the stored steps
    for (int i = 0; i <= positionsCounter - 1; i++) {
      if (data[1] == 0) {
        break;
      }
      stepper1.moveTo(theta1Array[i]);
      stepper2.moveTo(theta2Array[i]);
      stepper3.moveTo(phiArray[i]);
      stepper4.moveTo(zArray[i]);
      while (stepper1.currentPosition() != theta1Array[i] 
         || stepper2.currentPosition() != theta2Array[i] 
         || stepper3.currentPosition() != phiArray[i] 
         || stepper4.currentPosition() != zArray[i]) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
        stepper4.run();
      }
      if (i == 0) {
        gripperServo.write(gripperArray[i]);
      } else if (gripperArray[i] != gripperArray[i - 1]) {
        gripperServo.write(gripperArray[i]);
        delay(800);  // wait 0.8s for the servo to grab or drop - the servo is slow
      }

      //check for change in speed and acceleration or program stop
      if (Serial.available()) {
        content = Serial.readString();  // Read the incomding data from Processing
        // Extract the data from the string and put into separate integer variables (data[] array)
        for (int i = 0; i < 10; i++) {
          int index = content.indexOf(",");                     // locate the first ","
          data[i] = atol(content.substring(0, index).c_str());  //Extract the number from start to the ","
          content = content.substring(index + 1);               //Remove the number from the string
        }

        if (data[1] == 0) {
          break;
        }
        // change speed and acceleration while running the program
        stepper1.setSpeed(data[7]);
        stepper2.setSpeed(data[7]);
        stepper3.setSpeed(data[7]);
        stepper4.setSpeed(data[7]);

        stepper1.setAcceleration(data[8]);
        stepper2.setAcceleration(data[8]);
        stepper3.setAcceleration(data[8]);
        stepper4.setAcceleration(data[8]);
      }
    }
    /*
      // execute the stored steps in reverse
      for (int i = positionsCounter - 2; i >= 0; i--) {
      if (data[1] == 0) {
        break;
      }
      stepper1.moveTo(theta1Array[i]);
      stepper2.moveTo(theta2Array[i]);
      stepper3.moveTo(phiArray[i]);
      stepper4.moveTo(zArray[i]);
      while (stepper1.currentPosition() != theta1Array[i] || stepper2.currentPosition() != theta2Array[i] || stepper3.currentPosition() != phiArray[i] || stepper4.currentPosition() != zArray[i]) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
        stepper4.run();
      }
      gripperServo.write(gripperArray[i]);

      if (Serial.available()) {
        content = Serial.readString(); // Read the incomding data from Processing
        // Extract the data from the string and put into separate integer variables (data[] array)
        for (int i = 0; i < 10; i++) {
          int index = content.indexOf(","); // locate the first ","
          data[i] = atol(content.substring(0, index).c_str()); //Extract the number from start to the ","
          content = content.substring(index + 1); //Remove the number from the string
        }
        if (data[1] == 0) {
          break;
        }
      }
      }
    */
  }

  stepper1Position = data[2] * theta1AngleToSteps;
  stepper2Position = data[3] * theta2AngleToSteps;
  stepper3Position = data[4] * phiAngleToSteps;
  stepper4Position = data[5] * zDistanceToSteps;

  stepper1.setSpeed(data[7]);
  stepper2.setSpeed(data[7]);
  stepper3.setSpeed(data[7]);
  stepper4.setSpeed(data[7]);

  stepper1.setAcceleration(data[8]);
  stepper2.setAcceleration(data[8]);
  stepper3.setAcceleration(data[8]);
  stepper4.setAcceleration(data[8]);

  stepper1.moveTo(stepper1Position);
  stepper2.moveTo(stepper2Position);
  stepper3.moveTo(stepper3Position);
  stepper4.moveTo(stepper4Position);

  while (stepper1.currentPosition() != stepper1Position 
      || stepper2.currentPosition() != stepper2Position 
      || stepper3.currentPosition() != stepper3Position 
      || stepper4.currentPosition() != stepper4Position) {
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();
  }
  delay(100);
  gripperServo.write(data[6]);
  delay(300);
}

void serialFlush() {
  while (Serial.available() > 0) {  //while there are characters in the serial buffer, because Serial.available is >0
    Serial.read();                  // get one character
  }
}

void setHome() {
  // Homing Stepper4
  while (digitalRead(LIMIT_SWITCH_PIN_4) != 1) {
    stepper4.setSpeed(1500);
    stepper4.runSpeed();
    stepper4.setCurrentPosition(17000);  // When limit switch pressed set position to 0 steps
  }
  delay(20);
  stepper4.moveTo(10000);
  while (stepper4.currentPosition() != 10000) {
    stepper4.run();
  }

  // Homing Stepper3
  while (digitalRead(LIMIT_SWITCH_PIN_3) != 1) {
    stepper3.setSpeed(-1100);
    stepper3.runSpeed();
    stepper3.setCurrentPosition(-1662);  // When limit switch pressed set position to 0 steps
  }
  delay(20);
  stepper3.moveTo(0);
  while (stepper3.currentPosition() != 0) {
    stepper3.run();
  }

  // Homing Stepper2
  while (digitalRead(LIMIT_SWITCH_PIN_2) != 1) {
    stepper2.setSpeed(-1300);
    stepper2.runSpeed();
    stepper2.setCurrentPosition(-5420);  // When limit switch pressed set position to -5440 steps
  }
  delay(20);
  stepper2.moveTo(0);
  while (stepper2.currentPosition() != 0) {
    stepper2.run();
  }

  // Homing Stepper1
  while (digitalRead(LIMIT_SWITCH_PIN_1) != 1) {
    stepper1.setSpeed(-1200);
    stepper1.runSpeed();
    stepper1.setCurrentPosition(-3955);  // When limit switch pressed set position to 0 steps
  }
  delay(20);
  stepper1.moveTo(0);
  while (stepper1.currentPosition() != 0) {
    stepper1.run();
  }
}
