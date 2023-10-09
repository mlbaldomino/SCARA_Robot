/**
 * Martha Linares - StepperMotorDriver.h
 */

#ifndef STEPPER_MOTOR_DRIVER_H
#define STEPPER_MOTOR_DRIVER_H

#define MAX_STEPPER_SPEED 4000
#define DEFAULT_STEPPER_ACCELERATION 2000

#include <AccelStepper.h>
#include <Arduino.h>

class StepperMotorDriver
{
  private:
    AccelStepper *m_stepper;

    uint8_t m_limitSwitchPin;
    long    m_limitPosition;
    long    m_homePosition;
    long    m_stepperPosition;
    float   m_speed;
    float   m_acceleration;

  public:
    StepperMotorDriver(uint8_t stepperPin1, uint8_t stepperPin2, uint8_t limitSwitchPin);
    ~StepperMotorDriver();

  public:
    StepperMotorDriver* initialize(float speed, long limitPosition);
    StepperMotorDriver* setHome(long homePosition);
    StepperMotorDriver* goHome();

    void moveTo(long absolutePosition);
    void isMoving();
};

#endif