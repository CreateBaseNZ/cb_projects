#ifndef RobotArm_h
#define RobotArm_h

#include "Arduino.h"
#include "Geometry.h"
#include "Servo.h"
#include "BasicLinearAlgebra.h"
#include "Kinematics.h"

#define noOfJoints 4

class RobotArm
{
public:
    enum Direction
    {
        Up,
        Down,
        Left,
        Right,
        Forward,
        Backward,
        PitchUp,
        PitchDown,
        Stop
    };
    int servoLowerLimit[noOfJoints];
    int servoUpperLimit[noOfJoints];

    RobotArm();
    RobotArm(float linkLengths[noOfJoints + 1]);
    void ConfigurePins();
    void CalibrateServos();
    void Move(Direction inputDirection);

private:
    Servo servoMotors[noOfJoints];
    float linkLengths[noOfJoints + 1];
    float jointAngles[noOfJoints];

    float GetServoMicroseconds(int servoNumber);
    float GetServoDegrees(int servoNumber);
    float Mapf(float value, float fromLow, float fromHigh, float toLow, float toHigh);
    void ForwardKinematics(Matrix<4, 4> o[], float r[], float t[]);
    Matrix<noOfJoints, 1> GaussianElimination(Matrix<noOfJoints, noOfJoints> jacobian, Matrix<noOfJoints, 1> targetVelocity);
    Matrix<noOfJoints, noOfJoints> CalculateJacobian(Matrix<4, 4> transform[]);
    Matrix<noOfJoints, 1> InverseVelocityKinematics(Matrix<4, 4> o[], float r[], float t[], Matrix<noOfJoints, 1> targetVelocity);
};
#endif