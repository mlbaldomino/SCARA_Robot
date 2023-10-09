/**
 * Martha Linares - Arduino based SCARA Robot
 * base on Dejan Code, www.HowToMechatronics.com
 */

#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

#include "StepperMotorDriver.h"

#define LIMIT_SWITCH_PIN_1 11
#define LIMIT_SWITCH_PIN_2 10
#define LIMIT_SWITCH_PIN_3 9
#define LIMIT_SWITCH_PIN_4 A3

double x = 10.0;
double y = 10.0;
double L1 = 228.0; // L1 = 228.0 mm
double L2 = 136.5; // L2 = 136.5 mm
double theta1, theta2, phi, z;

int stepper1Position, stepper2Position, stepper3Position, stepper4Position;

const float theta1AngleToSteps = 44.444444;
const float theta2AngleToSteps = 35.555555;
const float phiAngleToSteps = 10.0;
const float zDistanceToSteps = 100.0;

byte inputValue[5] = {0};
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
int data[10] = {0};

int theta1Array[100] = {0};
int theta2Array[100] = {0};
int phiArray[100] = {0};
int zArray[100] = {0};
int gripperArray[100] = {0};

int positionsCounter = 0;

StepperMotorDriver *motorDriver1;
StepperMotorDriver *motorDriver2;
StepperMotorDriver *motorDriver3;
StepperMotorDriver *motorDriver4;

// Create servo object to control a servo
Servo gripperServo;


void setup()
{
    Serial.begin(115200);
    Serial.println("Running...");

    // Initialize the motor drivers.
    motorDriver4 = (new StepperMotorDriver(12, 13, LIMIT_SWITCH_PIN_4))
                    ->setSpeed(1500)
                    ->initialize(17000)
                    ->setHome(10000);

    motorDriver3 = (new StepperMotorDriver(4, 7, LIMIT_SWITCH_PIN_3))
                    ->setSpeed(-1100)
                    ->initialize(-1662)
                    ->setHome(0);

    motorDriver2 = (new StepperMotorDriver(3, 6, LIMIT_SWITCH_PIN_2))
                    ->setSpeed(-1300)
                    ->initialize(-5420)
                    ->setHome(0);

    motorDriver1 = (new StepperMotorDriver(2, 5, LIMIT_SWITCH_PIN_1))
                    ->setSpeed(-1200)
                    //->initialize(-7910)
                    ->initialize(-3955)
                    ->setHome(90 * theta1AngleToSteps);

    Serial.println("Initialized");

    // Configure gripper servo (pin, min, max)
    gripperServo.attach(A0, 600, 2500);

    // Set initial servo value - open gripper
    data[6] = 180;
    gripperServo.write(data[6]);

    delay(1000);

    data[2] = 90;
    data[5] = 100;
    goHome();
}

void loop()
{
    if (Serial.available())
    {
        content = Serial.readString(); // Read the incomding data from Processing
        // Extract the data from the string and put into separate integer variables (data[] array)
        for (int i = 0; i < 10; i++)
        {
            int index = content.indexOf(",");                                        // locate the first ","
            data[i] = atol(content.substring(0, index).c_str()); // Extract the number from start to the ","
            content = content.substring(index + 1);                            // Remove the number from the string
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
        if (data[0] == 1)
        {
            theta1Array[positionsCounter] = data[2] * theta1AngleToSteps; // store the values in steps = angles * angleToSteps variable
            theta2Array[positionsCounter] = data[3] * theta2AngleToSteps;
            phiArray[positionsCounter] = data[4] * phiAngleToSteps;
            zArray[positionsCounter] = data[5] * zDistanceToSteps;
            gripperArray[positionsCounter] = data[6];
            positionsCounter++;
        }

        // Clear data
        if (data[0] == 2)
        {
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
    while (data[1] == 1)
    {
        motorDriver1->setSpeed(data[7]);
        motorDriver2->setSpeed(data[7]);
        motorDriver3->setSpeed(data[7]);
        motorDriver4->setSpeed(data[7]);

        motorDriver1->setAcceleration(data[8]);
        motorDriver2->setAcceleration(data[8]);
        motorDriver3->setAcceleration(data[8]);
        motorDriver4->setAcceleration(data[8]);

        // execute the stored steps
        for (int i = 0; i <= positionsCounter - 1; i++)
        {
            if (data[1] == 0)
            {
                break;
            }
            motorDriver1->moveTo(theta1Array[i]);
            motorDriver2->moveTo(theta2Array[i]);
            motorDriver3->moveTo(phiArray[i]);
            motorDriver4->moveTo(zArray[i]);
            
            while (motorDriver1->isMoving()
                || motorDriver2->isMoving()
                || motorDriver3->isMoving()
                || motorDriver4->isMoving());

            if (i == 0)
            {
                gripperServo.write(gripperArray[i]);
            }
            else if (gripperArray[i] != gripperArray[i - 1])
            {
                gripperServo.write(gripperArray[i]);
                delay(800); // wait 0.8s for the servo to grab or drop - the servo is slow
            }

            // check for change in speed and acceleration or program stop
            if (Serial.available())
            {
                content = Serial.readString(); // Read the incomding data from Processing
                // Extract the data from the string and put into separate integer variables (data[] array)
                for (int i = 0; i < 10; i++)
                {
                    int index = content.indexOf(",");                                        // locate the first ","
                    data[i] = atol(content.substring(0, index).c_str()); // Extract the number from start to the ","
                    content = content.substring(index + 1);                            // Remove the number from the string
                }

                if (data[1] == 0)
                {
                    break;
                }
                // change speed and acceleration while running the program
                motorDriver1->setSpeed(data[7]);
                motorDriver2->setSpeed(data[7]);
                motorDriver3->setSpeed(data[7]);
                motorDriver4->setSpeed(data[7]);

                motorDriver1->setAcceleration(data[8]);
                motorDriver2->setAcceleration(data[8]);
                motorDriver3->setAcceleration(data[8]);
                motorDriver4->setAcceleration(data[8]);
            }
        }
        /*
            // execute the stored steps in reverse
            for (int i = positionsCounter - 2; i >= 0; i--) {
            if (data[1] == 0) {
                break;
            }
            motorDriver1->moveTo(theta1Array[i]);
            motorDriver2->moveTo(theta2Array[i]);
            motorDriver3->moveTo(phiArray[i]);
            motorDriver4->moveTo(zArray[i]);
            while (motorDriver1->currentPosition() != theta1Array[i] || motorDriver2->currentPosition() != theta2Array[i] || motorDriver3->currentPosition() != phiArray[i] || motorDriver4->currentPosition() != zArray[i]) {
                motorDriver1->run();
                motorDriver2->run();
                motorDriver3->run();
                motorDriver4->run();
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

    Serial.println("pos");
    Serial.println(stepper1Position);
    Serial.println(stepper2Position);
    Serial.println(stepper3Position);
    Serial.println(stepper4Position);
    Serial.println();

  /*  motorDriver1->setSpeed(data[7]);
    motorDriver2->setSpeed(data[7]);
    motorDriver3->setSpeed(data[7]);
    motorDriver4->setSpeed(data[7]);

    motorDriver1->setAcceleration(data[8]);
    motorDriver2->setAcceleration(data[8]);
    motorDriver3->setAcceleration(data[8]);
    motorDriver4->setAcceleration(data[8]);
*/
    motorDriver1->moveTo(stepper1Position);
    motorDriver2->moveTo(stepper2Position);
    motorDriver3->moveTo(stepper3Position);
    motorDriver4->moveTo(stepper4Position);


    bool move = false;
    do
    {
        move = motorDriver1->isMoving();
        move |= motorDriver2->isMoving();
        move |= motorDriver3->isMoving();
        move |= motorDriver4->isMoving();

        Serial.println("moving");
    }
    while (move);

    delay(100);
    gripperServo.write(data[6]);
    delay(300);
}

void serialFlush()
{
    while (Serial.available() > 0)
    {                                // while there are characters in the serial buffer, because Serial.available is >0
        Serial.read(); // get one character
    }
}

void goHome()
{
    motorDriver1->goHome();
    motorDriver2->goHome();
    motorDriver3->goHome();
    motorDriver4->goHome();
}
