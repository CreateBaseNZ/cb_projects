#include "Arduino.h"
#include "RobotArm.h"
#include "Servo.h"
#include "SoftwareSerial.h"

char bitData[100];
float data[20];
size_t numRead;
float linkLengths[] = {0.05, 0.105, 0.105, 0.045};
RobotArm robotArm(linkLengths);

// This function coverts a comma separated string into an array
void Parse(char input[], float output[])
{
  int count = 0;
  char temp[10];
  int tempCount = 0;
  for (size_t i = 0; i < numRead; i++)
  {
    if (input[i] != ',')
    {
      temp[tempCount] = input[i];
      tempCount++;
    }
    else
    {
      temp[tempCount] = '\0';
      output[count] = atof((char *)temp);
      temp[0] = '\0';
      tempCount = 0;
      count++;
    }
  }
}

void setup()
{
  Serial.begin(9600);
  robotArm.ConfigurePins();
  robotArm.CalibrateServos();
  pinMode(7, OUTPUT);
}

void loop()
{

  digitalWrite(7, HIGH); // Supplies power to Bluetooth Module

  // Read incomming serial data for commands
  if (Serial.available() > 0)
  {
    numRead = Serial.readBytesUntil('\n', bitData, sizeof(bitData) - 1);
    Parse(bitData, data);
  }

  // The data stored into the arrays correspond to a certain mode or controller ouput
  // Ill have to get back to you on what each array element represents.
  if (data[13] > 0)
  {
    // Mode 1
    for (size_t i = 0; i < noOfJoints; i++)
    {
      robotArm.servoMotors[i].write(data[i]);
    }
  }
  else if (data[14] > 0)
  {
    // Mode 2
    robotArm.Move(data[7], data[8], data[9], data[10], data[11], data[12]);
  }
  else if (data[15] > 0)
  {
    // Mode 3
  }
}
