#include "Arduino.h"
#include "RobotArm.h"
#include "SoftwareSerial.h"
#include "VarSpeedServo.h"

char bitData[100];
float data[20];
size_t numRead;
float linkLengths[] = {0.05, 0.105, 0.105, 0.045};
RobotArm robotArm(linkLengths);
float positions[10][4];
int noOfPositions=-1;
// This function coverts a comma separated string into an array
int Parse(char input[], float output[])
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

void EnterInputPos(){
  bool Entering=false;
  int index=0;
  while(!Entering){
    int length;
    if(Serial.available()){
      numRead = Serial.readBytesUntil('\n', bitData, sizeof(bitData) - 1);
      length=Parse(bitData,data);
      if(noOfPositions==-1){
        if (length==1 && data[0]>0 && data[0]<=10){
          noOfPositions=data[0];
        }
      }else{
        if(length==4){
          for(int i=0;i<4;i++){
            positions[index][i]=data[i];
          }
          index++;
          if(index==noOfPositions){
            Entering=true;
          }
        }
      }
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
  robotArm.Move_position_4link(0.15,0.155,180 ,180);
  int pin[3]={6,14,18};
  robotArm.ConfigureUltraSonic(pin,3);
  delay(1000);
  //EnterInputPos();
}
int num=0;
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
  //robotArm.Move(0,0,0.0025,0,0,0);

  //Hand control mode
  //robotArm.HandControl();


  //Skeleton for basketball
  // for(int i=0;i<noOfPositions;i++){
  //     robotArm.Move_position_4link(positions[i][0],positions[i][1],positions[i][2],positions[i][3]);
  //     while(!robotArm.DetectPassage()){
  //       delay(15);
  //     }
  // }

}
