#include "Arduino.h"
#include "RobotArm.h"
#include "Servo.h"

float linkLengths[] = {0.05, 0.105, 0.105, 0.045};
RobotArm robotArm(linkLengths);

void setup()
{
  Serial.begin(9600);
  robotArm.ConfigurePins();
  robotArm.CalibrateServos();
}

void loop()
{
  if (Serial.available() > 0)
  {
    int input = Serial.read();
    switch (input)
    {
    case 'w':
      robotArm.Move(robotArm.Forward);
      Serial.println("Forward");
      break;
    case 's':
      robotArm.Move(robotArm.Backward);
      Serial.println("Backward");
      break;
    case 'a':
      robotArm.Move(robotArm.Left);
      Serial.println("Left");
      break;
    case 'd':
      robotArm.Move(robotArm.Right);
      Serial.println("Right");
      break;
    case 'z':
      robotArm.Move(robotArm.Up);
      Serial.println("Up");
      break;
    case 'x':
      robotArm.Move(robotArm.Down);
      Serial.println("Down");
      break;
    case 'q':
      robotArm.Move(robotArm.PitchUp);
      Serial.println("Pitch Up");
      break;
    case 'e':
      robotArm.Move(robotArm.PitchDown);
      Serial.println("Pitch Down");
      break;
    default:
      robotArm.Move(robotArm.Stop);
      break;
    }
  }
}
