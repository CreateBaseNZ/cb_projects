#ifndef RobotArm_h
#define RobotArm_h

#include "Arduino.h"
#include "Geometry.h"
#include "BasicLinearAlgebra.h"
#include "VarSpeedServo.h"
#include "Helper.h"

#define noOfJoints 4
#define noOfSonar 5
class RobotArm
{
public:
    float jointAngles[noOfJoints];
    VarSpeedServo servoMotors[noOfJoints];
    bool DetectPassage();
    void HandControl();
    float findDistance(int sonarNo);
    void DrawCircle(float raduis, float z);
    void DrawSquare(float Length,float z);
    void ResetDraw();
    RobotArm();
    RobotArm(float linkLengths[noOfJoints + 1]);
    void AttachMotors();
    void DetachMotors();
    void ConfigurePins();
    void ConfigureUltraSonic(int pins[],int sonars);
    void CalibrateServos();
    void Move(float vx, float vy, float vz, float wx, float wy, float wz);
    bool Move_position_4link(float r,float z, float theta, float alpha);
    float GetServoDegrees(int servoNumber);
    float Mapf(float value, float fromLow, float fromHigh, float toLow, float toHigh);
        Matrix<noOfJoints, noOfJoints> CalculateJacobian(Matrix<4, 4> transform[]);
    void ForwardKinematics(Matrix<4, 4> o[], float r[], float t[]);
    Matrix<noOfJoints, 1> GaussianElimination(Matrix<noOfJoints, noOfJoints> jacobian, Matrix<noOfJoints, 1> targetVelocity);

private:
    int servoLowerLimit[noOfJoints];
    int servoUpperLimit[noOfJoints];
    float linkLengths[noOfJoints + 1];
    int sonarTrigPin[noOfSonar];        
    bool Ultrasonic=false;
    int DrawValue;
    bool DrawingDone;
    int sonarUsed=0;
    Matrix<noOfJoints, 1> InverseVelocityKinematics( float r[], float t[], Matrix<noOfJoints, 1> targetVelocity);
    void FindLocation(float loaction[]);
    float roundXdp(float input,int decimalPlaces);
    float GetServoMicroseconds(int servoNumber);
    float torad(float angle);
    float todeg(float angle);
    float Circle_round(float input);
    float round2dp(float input);
    void findAngles1_3(float angles[],float theta,float alpha,float C);

};
#endif