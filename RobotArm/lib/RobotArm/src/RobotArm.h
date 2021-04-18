#ifndef RobotArm_h
#define RobotArm_h

#include "Arduino.h"
#include "Geometry.h"
#include "BasicLinearAlgebra.h"
#include "VarSpeedServo.h"
#include "Helper.h"

#define noOfJoints 4
#define noOfSonar 5
#define noOfPhototransitors 5


class RobotArm
{
public:
    float jointAngles[noOfJoints];
    VarSpeedServo servoMotors[noOfJoints];
    //Cartesian coordinate system
    bool Move_position_cart(float x, float y, float z);
    bool Move_position_cart_theta(float final_x, float final_y, float final_z, float theta_deg);
    void getToPosition_cart(float x, float y, float z);
    void getToPosition_cart_theta(float x, float y, float z,float theta);
    bool Move_x_units(float x_ch, float y_ch, float z_ch);
    bool Move_x_units_theta(float x_ch, float y_ch, float z_ch, float theta_deg_ch);
    void getToPositionShift_cart(float x_ch, float y_ch, float z_ch);
    void getToPositionShift_cart_theta(float x_ch, float y_ch, float z_ch, float theta_deg_ch);
    //Cylindrical coordinate system
    bool Move_position_cylinder(float r, float alpha_deg,float z);
    bool Move_position_cylinder_theta(float r, float alpha_deg,float z, float theta_deg);
    void getToPosition_cylinder(float r, float alpha_deg,float z);
    void getToPosition_cylinder_theta(float r, float alpha_deg,float z, float theta_deg);
    //Spherical coordinate system 
    bool Move_position_spherical(float R, float alpha_deg,float phi_deg);
    bool Move_position_spherical_theta(float R, float alpha_deg,float phi_deg, float theta_deg);
    void getToPosition_spherical(float R, float alpha_deg,float phi_deg);
    void getToPosition_spherical_theta(float R, float alpha_deg, float phi_deg, float theta_deg);

    bool DetectPassage();
    void HandControl();
    float findDistance(int sonarNo);
    void DrawCircle(float raduis, float z);
    void DrawSquare(float Length,float z);
    void ResetDraw();
    RobotArm();
    RobotArm(float linkLengths[noOfJoints + 1]);
    void AttachMotors(int pins[]);
    void DetachMotors();
    void ConfigurePins(int pins[]);
    void ConfigureUltraSonic(int pins[],int sonars);
    void CalibrateServos();
    void Move(float vx, float vy, float vz, float wx, float wy, float wz);
    
    float GetServoDegrees(int servoNumber);
    float Mapf(float value, float fromLow, float fromHigh, float toLow, float toHigh);
    void ConfigureBasketBall(int pins[],int number,int limit[]);
    void ForwardKinematics(Matrix<4, 4> o[], float r[], float t[]);
    int targetAngle[4] = {0, 0, 0, 0};
private:
    int noOfTransitors=0;
    void endEffecterPos(float location[]);
    int phototransisorPins[noOfPhototransitors];
    int limits[noOfPhototransitors];
    int servoLowerLimit[noOfJoints];
    int servoUpperLimit[noOfJoints];
    float linkLengths[noOfJoints + 1];
    int sonarTrigPin[noOfSonar];        
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
    Matrix<noOfJoints, 1> GaussianElimination(Matrix<noOfJoints, noOfJoints> jacobian, Matrix<noOfJoints, 1> targetVelocity);
    Matrix<noOfJoints, noOfJoints> CalculateJacobian(Matrix<4, 4> transform[]);
    float dotProduct(float v1[], float v2[], int size);
    float vectorLength(float v[], int size);
    float findAngle(float v1[], float v2[], int size);
};
#endif