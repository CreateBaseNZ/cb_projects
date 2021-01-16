#ifndef RobotArm_h
#define RobotArm_h

#include "Arduino.h"
#include "Geometry.h"
#include "Servo.h"
#include "BasicLinearAlgebra.h"
#include "Helper.h"

#define noOfJoints 4
#define noOfSonar 3
class RobotArm
{
public:
    float jointAngles[noOfJoints];
    Servo servoMotors[noOfJoints];
    void HandControl(float time);
    float findDistance(int sonarNo);
    void DrawCircle(float raduis, float z);
    void DrawSquare(float Length,float z);
    void ResetDraw();
    RobotArm();
    RobotArm(float linkLengths[noOfJoints + 1]);
    void AttachMotors();
    void DetachMotors();
    void ConfigurePins();
    void ConfigureUltraSonic(int pins[noOfSonar]);
    void CalibrateServos();
    void Move(float r_dot, float z_dot, float theta_dot, float alpha_dot, float wy, float wz,float time);
    bool Move_position_4link(float r,float z, float theta, float alpha);
    float GetServoDegrees(int servoNumber);
    float Mapf(float value, float fromLow, float fromHigh, float toLow, float toHigh);

private:
    int servoLowerLimit[noOfJoints];
    int servoUpperLimit[noOfJoints];
    float linkLengths[noOfJoints + 1];
    int sonarTrigPin[noOfSonar];        
    bool Ultrasonic=false;
    int DrawValue;
    bool DrawingDone;

    void FindLocation(float loaction[]);
    float roundXdp(float input,int decimalPlaces);
    float GetServoMicroseconds(int servoNumber);
    float torad(float angle);
    float todeg(float angle);
    float HalfCircle_round(float input);
    float Circle_round(float input);
    float round2dp(float input);
    void ForwardKinematics(Matrix<4, 4> o[], float r[], float t[]);
    Matrix<noOfJoints, 1> GaussianElimination(Matrix<noOfJoints, noOfJoints> jacobian, Matrix<noOfJoints, 1> targetVelocity);
    Matrix<noOfJoints, noOfJoints> CalculateJacobian(Matrix<4, 4> transform[]);
    Matrix<noOfJoints, 1> InverseVelocityKinematics(Matrix<4, 4> o[], float r[], float t[], Matrix<noOfJoints, 1> targetVelocity);
};
#endif