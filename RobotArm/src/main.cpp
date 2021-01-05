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


float torad(float angle){
    return angle*M_PI/180;
}

int alpha_deg=0;
void setup()
{
  Serial.begin(9600);
  robotArm.ConfigurePins();
  robotArm.CalibrateServos();
  pinMode(7, OUTPUT);
  robotArm.ResetDraw();
  delay(1000);
  Serial<<"Drawing Starting\n";
}

float last_t=0;
void loop()
{


  float r, z,  theta_deg;
  float angle1=50,angle2=50,angle3=0;
  theta_deg=angle1+angle2+angle3;
  theta_deg=270;
  r=2*.105*cos(torad(35));//linkLengths[1]*cos(torad(angle1))+linkLengths[2]*cos(torad(angle1+angle2))+linkLengths[3]*cos(torad(angle2+angle1+angle3));
  z=0.005;//linkLengths[0]+linkLengths[1]*sin(torad(angle1))+linkLengths[2]*sin(torad(angle1+angle2))+linkLengths[3]*sin(torad(angle1+angle2+angle3));
  robotArm.DrawCircle(r,z);

  // robotArm.Move_position_4link(r,z,theta_deg,alpha_deg);
  // delay(500);

  // digitalWrite(7, HIGH); // Supplies power  to Bluetooth Module

  // // Read incomming serial data for commands
  // if (Serial.available() > 0)
  // {
  //   numRead = Serial.readBytesUntil('\n', bitData, sizeof(bitData) - 1);
  //   Parse(bitData, data);
  // }

  // // The data stored into the arrays correspond to a certain mode or controller ouput
  // // Ill have to get back to you on what each array element represents.
  // if (data[13] > 0)
  // {
  //   // Mode 1
  //   for (size_t i = 0; i < noOfJoints; i++)
  //   {
  //     robotArm.servoMotors[i].write(data[i]);
  //   }
  // }
  // else if (data[14] > 0)
  // {
  //   // Mode 2
  //   robotArm.Move(data[7], data[8], data[9], data[10], data[11], data[12]);
  // }
  // else if (data[15] > 0)
  // {
  //   // Mode 3
  // }
}
