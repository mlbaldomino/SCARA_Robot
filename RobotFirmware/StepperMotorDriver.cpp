/**
 * Martha Linares - StepperMotorDriver.cpp
 */

#include "StepperMotorDriver.h"

StepperMotorDriver::StepperMotorDriver(uint8_t stepperPin1, uint8_t stepperPin2, uint8_t limitSwitchPin)
{
    // Initialize the stepper motor
    m_limitSwitchPin = limitSwitchPin;
    m_stepper = new AccelStepper(AccelStepper::DRIVER, stepperPin1, stepperPin2);

    // Set stepper motor max speed
    m_stepper->setMaxSpeed(MAX_STEPPER_SPEED);

    // Set stepper motor default acceleration
    m_stepper->setAcceleration(DEFAULT_STEPPER_ACCELERATION);

    // Set internal pullups for input limit switch
    pinMode(m_limitSwitchPin, INPUT_PULLUP);
}

StepperMotorDriver::~StepperMotorDriver()
{
    delete m_stepper;
    m_stepper = nullptr;
}

StepperMotorDriver* StepperMotorDriver::initialize(long limitPosition)
{
    // Homing Stepper
    m_limitPosition = limitPosition;
    while (digitalRead(m_limitSwitchPin) != HIGH)
    {
        m_stepper->setSpeed(m_speed);
        m_stepper->runSpeed();
        m_stepper->setCurrentPosition(m_limitPosition); // When limit switch pressed set position to limitPosition steps
    }

    delay(20);

    return this;
}

StepperMotorDriver* StepperMotorDriver::setSpeed(float speed)
{
    m_speed = speed;
    m_stepper->setSpeed(m_speed);

    return this;
}

StepperMotorDriver* StepperMotorDriver::setAcceleration(float acceleration)
{
    m_acceleration = acceleration;
    m_stepper->setAcceleration(m_acceleration);

    return this;
}

StepperMotorDriver* StepperMotorDriver::setHome(long homePosition)
{
    m_homePosition = homePosition;
    return this;
}

StepperMotorDriver* StepperMotorDriver::goHome()
{
    m_stepper->moveTo(m_homePosition);
    while (m_stepper->currentPosition() != m_homePosition)
    {
        m_stepper->run();
    }

    return this;
}

void StepperMotorDriver::moveTo(long absolutePosition)
{
    m_stepperPosition = absolutePosition;
    m_stepper->moveTo(m_stepperPosition);
}

bool StepperMotorDriver::isMoving()
{
    // TODO: Improve this function.
    bool arrived = m_stepper->currentPosition() != m_stepperPosition;

    bool isStillRunning = m_stepper->run();

    return arrived;
}