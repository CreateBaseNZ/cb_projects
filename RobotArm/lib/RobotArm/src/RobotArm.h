#ifndef RobotArm_h
#define RobotArm_h

#include "Arduino.h"
#include "Geometry.h"
#include "Servo.h"
#include "BasicLinearAlgebra.h"
#include "Helper.h"

#define noOfJoints 4

class RobotArm
{
public:
    float jointAngles[noOfJoints];
    Servo servoMotors[noOfJoints];

    RobotArm();
    RobotArm(float linkLengths[noOfJoints + 1]);
    void AttachMotors();
    void DetachMotors();
    void ConfigurePins();
    void CalibrateServos();
    void Move(float vx, float vy, float vz, float wx, float wy, float wz);
    void Move_position_4link(float r,float z, float theta, float alpha);
    float GetServoDegrees(int servoNumber);
    float Mapf(float value, float fromLow, float fromHigh, float toLow, float toHigh);

private:
    int servoLowerLimit[noOfJoints];
    int servoUpperLimit[noOfJoints];
    float linkLengths[noOfJoints + 1];

    float GetServoMicroseconds(int servoNumber);
    void ForwardKinematics(Matrix<4, 4> o[], float r[], float t[]);
    Matrix<noOfJoints, 1> GaussianElimination(Matrix<noOfJoints, noOfJoints> jacobian, Matrix<noOfJoints, 1> targetVelocity);
    Matrix<noOfJoints, noOfJoints> CalculateJacobian(Matrix<4, 4> transform[]);
    Matrix<noOfJoints, 1> InverseVelocityKinematics(Matrix<4, 4> o[], float r[], float t[], Matrix<noOfJoints, 1> targetVelocity);
};
#endif