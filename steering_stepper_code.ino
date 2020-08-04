// Project: RoboCup
// Code for the steering feature as well as the control of the arm with the stepper motor
// Thomas Courcy
// 08/03/2020

#include <Servo.h>
#include <Stepper.h>

Servo steeringServo;

const float STEPS_PER_REV = 32;
const float GEAR_RED = 64;
const float STEPS_PER_OUT_REV = STEPS_PER_REV * GEAR_RED;
Stepper steppermotor(STEPS_PER_REV, 10, 12, 11, 13);

void setup()
{
  steeringServo.attach(9);
}

void loop()
{
  // CALL FUNCTION WHEN NEEDED
}

void steerDrink()
{
  for (int numberOfSteer = 0; numberOfSteer < 5; numberOfSteer++)
  {
    steeringServo.write(10); // CHANGE VALUE WHEN TESTING
    delay(500);
    steeringServo.write(100); // CHANGE VALUE WHEN TESTING
    delay(500);
  }
}

void armUp()
{
  steppermotor.setSpeed(700);
  steppermotor.step(10); // CHANGE VALUE WHEN TESTING
}

void armDown()
{
  steppermotor.setSpeed(700);
  steppermotor.step(-10); // CHANGE VALUE WHEN TESTING
}
