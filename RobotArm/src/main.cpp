#include "Arduino.h"
#include "RobotArm.h"
#include "SoftwareSerial.h"
#include "VarSpeedServo.h"

float linkLengths[] = {0.05, 0.105, 0.105, 0.045};
RobotArm robotArm(linkLengths);
float positions[10][3];
int noOfPositions=-1;
// This function coverts a comma separated string into an array
int Parse(char input[], float output[],size_t numRead)
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
  temp[tempCount] = '\0';
  output[count] = atof((char *)temp);
  return count+1;

}

int EnterInputPos(float pos[10][3]){
  bool Entering=false;
  int index=0;
  int totalPositions=-1;
  Serial<<"How many different positions do you want?\n";
  while(!Entering){
    if(Serial.available()){
      char bitData[100];
      float data[20];
      size_t numRead;
      numRead = Serial.readBytesUntil('\n', bitData, sizeof(bitData) - 1);
      int length=Parse(bitData,data,numRead);
      if(totalPositions==-1){
        if (length==1 && data[0]>0 && data[0]<=10){
          totalPositions=data[0];
          Serial<<"Enter Postion Number 1:\n";
        }else{
          Serial<<"Enter a number between 1 and 10...\n";
        }
      }else{
        if(length==3){
          if(robotArm.Move_position_4link(data[0],data[1],0,data[2])){
            for(int i=0;i<3;i++){
              pos[index][i]=data[i];
            }
            Serial<<"Position Number "<<(index+1)<<": r= "<<data[0]<<", z= "<<data[1]<<", beta= 0, alpha= "<<data[2]<<"\n";
            index++;
            if(index==totalPositions){
              Entering=true;
            }else{
              Serial<<"Enter Position Number "<<(index+1)<<":\n";
            }
          }else{
            Serial<<"Enter an alternative position Number "<<(index+1)<<": \n";
          }
        }else{
          Serial<<"The inputs should be in the form= r, z, alpha\n";
        }
      }
    } 
  }
  return totalPositions;
}
float torad(float angle){
    return angle*M_PI/180;
}

float t=0,last_t;
int alpha_deg=0;
void setup()
{
 
  int motorPins[4]={2,3,4,5};
  Serial.begin(9600);
  robotArm.ConfigurePins(motorPins); 
  
  int pins[3]={15,18,19},limit[3]={180,200,140};
  robotArm.ConfigureBasketBall(pins,3,limit);


 // int pin[3]={6,14,18};
 // robotArm.ConfigureUltraSonic(pin,3);

  robotArm.Move_position_4link(0.15,0.155,0 ,0);
  //delay(1000);
  // noOfPositions=EnterInputPos(positions);
  // Serial<<"Start Shooting!!\n";
}

void loop()
{
  //This Section is for position controller testing
  // if(Serial.available()){
  //   numRead = Serial.readBytesUntil('\n', bitData, sizeof(bitData) - 1);
  //   float out[5];
  //   Parse(bitData,out);
  //   robotArm.Move_position_4link(out[0],out[1],out[2] ,out[3]);
  // }

  //Testing Inverse Kinematics
  // Matrix<4, 4> o[noOfJoints+1];
  // float r[4]={0,torad(0),torad(0),torad(0)};
  // robotArm.ForwardKinematics(o, r, linkLengths);
  // Serial<<o[4]<<"\n";
  // Matrix<noOfJoints,noOfJoints> VelForwardKinematics=robotArm.CalculateJacobian(o);
  // Serial<<VelForwardKinematics<<"\n";
  // if(VelForwardKinematics.Det()!=0){
  //   Matrix<noOfJoints,noOfJoints> VelInverseKinematics=VelForwardKinematics.Inverse();
  //   Serial<<VelInverseKinematics<<"\n";
  //   Matrix<4,1> Targert={0,0,1,0};
  //   Serial<<(VelInverseKinematics*Targert)<<"\n";
  // }else{
  //   Matrix<noOfJoints,1> zeroVector = {0, 0, 0, 0};
  //   Serial<<"too Much\n";
  // }
  // while(1){
  // }  
  

  //Test Move command
  //robotArm.Move(0.005,0,0,0,0,0);

  //Hand control mode
  //robotArm.HandControl();
  
  //Skeleton for basketball
  // for(int i=0;i<noOfPositions;i++){
  //   if(robotArm.Move_position_4link(positions[i][0],positions[i][1],0,positions[i][2])){
  //     delay(2000);
  //     while(!robotArm.DetectPassage()){
  //       delay(1);
  //     }
  //   }    
  // }
  robotArm.DetectPassage();
}
