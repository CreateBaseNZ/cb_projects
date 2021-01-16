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

float t=0,last_t;
int alpha_deg=0;
void setup()
{
  Serial.begin(9600);
  robotArm.ConfigurePins();
 // robotArm.servoMotors[0].write(90);
//  robotArm.CalibrateServos();
//   pinMode(7, OUTPUT);
   robotArm.Move_position_4link(0.15,0.155,180 ,180);
   int pin[3]={6,18,14};
robotArm.ConfigureUltraSonic(pin);
//   robotArm.ResetDraw();
//   delay(1000);
//   Serial<<"Drawing Starting\n";
//   t=0;
//   last_t=millis();
}

void loop()
{
  //  t=(millis()-last_t)/1000;;
  // if(t>0.1){
  //   last_t=millis();
  //   Serial<<"Time: "<<t<<"\n";
  //   robotArm.HandControl(t);
  // }

  // float r, z,  theta_deg;
  // float angle1=50,angle2=50,angle3=0;
  // theta_deg=angle1+angle2+angle3;
  // theta_deg=180;
  // float maxr=2*0.105*cos(torad(35))+0.045;
  // r=maxr*cos(torad(45));//linkLengths[1]*cos(torad(angle1))+linkLengths[2]*cos(torad(angle1+angle2))+linkLengths[3]*cos(torad(angle2+angle1+angle3));
  // z=0.05+0.21*sin(torad(35));//linkLengths[0]+linkLengths[1]*sin(torad(angle1))+linkLengths[2]*sin(torad(angle1+angle2))+linkLengths[3]*sin(torad(angle1+angle2+angle3));
    
  //   robotArm.DrawSquare(r,z);
    

    //robotArm.DrawCircle(2*cos(M_PI_4)*0.105,0.005);
//float angle1=0,angle2=-90,angle3=90;
   // float r, z,  theta_deg,  alpha_deg=135;

   // theta_deg=180;//angle1+angle2+angle3;
   // r=2*0.105+0.045;//abs(linkLengths[1]*cos(torad(angle1))+linkLengths[2]*cos(torad(angle1+angle2))+linkLengths[3]*cos(torad(angle2+angle1+angle3)));
//  z=0.05;//linkLengths[0]+linkLengths[1]*sin(torad(angle1))+linkLengths[2]*sin(torad(angle1+angle2))+linkLengths[3]*sin(torad(angle1+angle2+angle3));
  // robotArm.Move_position_4link(r,z,theta_deg,alpha_deg);
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
