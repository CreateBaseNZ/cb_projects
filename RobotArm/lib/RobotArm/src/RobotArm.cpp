#include "RobotArm.h"

RobotArm::RobotArm()
{
}

RobotArm::RobotArm(float linkLengths[])
{
    if (sizeof(*linkLengths) != noOfJoints + 1)
    {
        Serial.println("Bad initialisation of Robot Arm");
    }
    for (int i = 0; i < noOfJoints + 1; i++)
    {
        this->linkLengths[i] = linkLengths[i];
    }
}

void RobotArm::AttachMotors()
{
    for (int i = 0; i < noOfJoints; i++)
    {
        servoMotors[i].attach(i + 2, 500, 2500);
    }
}

void RobotArm::DetachMotors()
{
    for (int i = 0; i < noOfJoints; i++)
    {
        servoMotors[i].detach();
    }
}

void RobotArm::ConfigurePins()
{
    for (int i = 0; i < noOfJoints; i++)
    {
        Serial << "Configuring Pin: " << i << "\n";
        servoMotors[i].attach(i + 2);
        if (i == 0)
        {
            servoMotors[i].writeMicroseconds(500);
        }
        else
        {
            servoMotors[i].writeMicroseconds(1500);
        }
    }
}

// Calibration sequence takes the servos to their limits and records
// the positional feedback value
void RobotArm::CalibrateServos()
{
    Serial.println("Calibrating...");
    for (int i = 0; i < noOfJoints; i++)
    {
        Serial << "Calirbrating Lower: " << i << "\n";

        servoMotors[i].writeMicroseconds(500);
        // if (i == 1)
        // {
        //     servoMotors[i + 1].writeMicroseconds(2500);
        // }
        delay(3000);
        char input ='s';
        while(input!='d'){
            if(Serial.available()){
                input=Serial.read();
            }
        }
        servoLowerLimit[i] = analogRead(i);
        servoMotors[i].writeMicroseconds(1500);
    }

    delay(3000);

    for (int i = 0; i < noOfJoints; i++)
    {
        Serial << "Calirbrating higher: " << i << "\n";


        servoMotors[i].writeMicroseconds(2500);
        // if (i == 1)
        // {
        //     servoMotors[i + 1].writeMicroseconds(500);
        // }
        delay(3000);
        char input ='s';

        while(input!='d'){
            if(Serial.available()){
                input=Serial.read();
            }
        }
        servoUpperLimit[i] = analogRead(i);
        servoMotors[i].writeMicroseconds(1500);
    }

    Serial.println("Calibration Complete");
}

void RobotArm::ResetDraw(){
    DrawValue=0;
    DrawingDone=false;
}

float RobotArm::torad(float angle){
    return angle*M_PI/180;
}

float RobotArm::todeg(float angle){
    return angle*180/M_PI;
}

float RobotArm::HalfCircle_round(float input){
    input=round(input/0.01)*0.01;
    while(input<-M_PI){
        input+=2*M_PI;
    }
    input=todeg(input);
    input=(int(input+90))%180;
    return torad(input-90);
}

float RobotArm::round2dp(float input){
    return round(input/0.01)*0.01;
}

float RobotArm::Circle_round(float input){
    input=round(input/0.01)*0.01;
    while(input<-M_PI){
        input+=2*M_PI;
    }
    input=todeg(input);
    input=(int(input+180))%360;
    return torad(input-180);
}



void RobotArm::DrawCircle(float raduis, float z){
    int interval= 360/45;
    if(!DrawingDone){
        if(Move_position_4link(raduis,z,270,DrawValue*interval)){
            bool notReached=false;
            for(int i=0;i<noOfJoints && !notReached;i++){
                float error=servoMotors[i].read()-GetServoDegrees(i);
                Serial<<"Motor "<<i<<": "<<servoMotors[i].read()<<", "<<GetServoDegrees(i)<<" with error of "<<abs(error)<<"\n";
                if(abs(error)>4){
                    notReached=true;
                }
            }
            if(!notReached){
                if(DrawValue*interval>=360){
                    DrawingDone=true;
                    Serial<<"Drawing Done\n";
                    return;
                }else{
                    DrawValue++;
                    Serial<<"Point "<<DrawValue<<"\n";
                }
            }
        }else{
            Serial<<"Drawing Can't be Done\n";
            DrawingDone=true;
            return;
        }
    }
}



bool RobotArm::Move_position_4link(float r,float z, float theta_deg, float alpha_deg){
    float alpha=alpha_deg*M_PI/180;
    float theta=theta_deg*M_PI/180;
    float R,C,a,b,angles[4],cosAngle_2=0.0; 


    alpha=Circle_round(alpha);
    angles[0]=atan(sin(alpha)/ cos(alpha));
    
    R=r-linkLengths[3]*cos(theta);
    C=z-linkLengths[0]-linkLengths[3]*sin(theta);
    cosAngle_2=(float)((pow(R,2)+pow(C,2)-pow(linkLengths[1],2)-pow(linkLengths[2],2))/(2*linkLengths[2]*linkLengths[1]));
    if(abs(cosAngle_2)>1){
        Serial<<"Cant Reach Point\n";
        return false;
    }
    angles[2]=round2dp(acos(cosAngle_2));

    if(abs(alpha)<=M_PI_2){
        angles[2]=-angles[2];
    }

    a=linkLengths[2]*sin(angles[2]);
    b=linkLengths[1]+linkLengths[2]*cos(angles[2]);
    angles[1]=round2dp(atan2(C,sqrt(pow(a,2)+pow(b,2)-pow(C,2)))-atan2(a,b));
    angles[3]=HalfCircle_round(theta-angles[2]-angles[1]);

    if(angles[1]>M_PI ||angles[1]<0||abs(angles[3])>M_PI_2){
        angles[1]=round2dp(atan2(C,-sqrt(pow(a,2)+pow(b,2)-pow(C,2)))-atan2(a,b));
        angles[3]=HalfCircle_round(theta-angles[2]-angles[1]);
    }

    for(int i=0;i<noOfJoints;i++){
        angles[i]=round(angles[i]/0.01)*0.01;

        if(i==1){
            if (angles[i]>M_PI ||angles[i]<0){
                Serial<<"Cant Reach Point\n";
                return false;
            }
        }else{
            if (abs(angles[i])>M_PI_2){
                Serial<<"Cant Reach Point\n";
                return false;
            }
        }
    }

    for( int i=0;i<noOfJoints;i++){
        if(i!=1){
            angles[i]+=M_PI_2;
        }
        angles[i]=angles[i]*180/M_PI;
        servoMotors[i].write(angles[i]);
    }
  //  Serial<<angles[0]<<", "<<angles[1]<<", "<<angles[2]<<", "<<angles[3]<<"\n";

    return true;
    
}

// Uses velocity IK to determine actuation for motors
void RobotArm::Move(float vx, float vy, float vz, float wx, float wy, float wz)
{
    Matrix<4, 1> targetVelocity = {vx, vy, vz, wx};
    targetVelocity *= 2;

    float temp;
    for (int i = 0; i < noOfJoints; i++)
    {
        temp = GetServoDegrees(i);
        if (i > 0)
        {
            temp -= 90;
        }
        jointAngles[i] = temp*M_PI_2/90;
    }

    Matrix<4, 4> armTransform[noOfJoints + 1];
    Matrix<noOfJoints, 1> servoVelocity = InverseVelocityKinematics(armTransform, jointAngles, linkLengths, targetVelocity);

    for (int i = 0; i < noOfJoints; i++)
    {
        servoMotors[i].writeMicroseconds(servoMotors[i].readMicroseconds() + servoVelocity(i));
    }
}

float RobotArm::GetServoDegrees(int servo)
{
    return (Mapf(analogRead(servo), servoLowerLimit[servo], servoUpperLimit[servo], 0, 180));
}

float RobotArm::Mapf(float value, float fromLow, float fromHigh, float toLow, float toHigh)
{
    return ((value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow);
}



// Performs forward kinematics on the arm to determine the end effectors transformation matrix
void RobotArm::ForwardKinematics(Matrix<4, 4> o[], float r[], float t[])
{
    o[0] = {1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1};

    for (int i = 0; i <= noOfJoints; i++)
    {
        if ((i == 0) || ((noOfJoints == 6) && ((i == 3) || (i == 5))))
        {
            o[i + 1] = o[i] * Rotate_Z(r[i]) * Translate_XYZ(0, 0, t[i]);
        }
        else
        {
            o[i + 1] = o[i] * Rotate_X(r[i]) * Translate_XYZ(0, 0, t[i]);
        }
    }
}

// Solves systems of equations through gaussian elimination method.
Matrix<noOfJoints, 1> RobotArm::GaussianElimination(Matrix<noOfJoints, noOfJoints> jacobian, Matrix<noOfJoints, 1> targetVelocity)
{
    Matrix<noOfJoints, noOfJoints + 1> augment = jacobian || targetVelocity;
    float scalar;

    // Get zeros in lower left
    for (int i = 0; i < noOfJoints; i++)
    {
        scalar = augment(i, i);
        if (scalar == 0)
        {
            break;
        }
        for (int j = 0; j <= noOfJoints; j++)
        {
            augment(i, j) /= scalar;
        }

        for (int j = i + 1; j < noOfJoints; j++)
        {
            scalar = augment(j, i);
            if (scalar == 0)
            {
                break;
            }
            for (int k = 0; k <= noOfJoints; k++)
            {
                augment(j, k) -= scalar * augment(i, k);
            }
        }
    }

    // Get zeros in top right
    for (int i = noOfJoints - 1; i > 0; i--)
    {

        for (int j = i - 1; j >= 0; j--)
        {
            scalar = augment(j, i);
            for (int k = 0; k <= noOfJoints; k++)
            {
                augment(j, k) -= scalar * augment(i, k);
            }
        }
    }

    // Output solved matrix
    Matrix<noOfJoints, 1> jointVelocity;
    for (int i = 0; i < noOfJoints; i++)
    {
        jointVelocity(i) = augment(i, noOfJoints);
    }

    return jointVelocity;
}







// Calculates jacobian matrix for the robotic arm required for velocity IK
Matrix<noOfJoints, noOfJoints> RobotArm::CalculateJacobian(Matrix<4, 4> transform[])
{
    Matrix<noOfJoints, noOfJoints> jacobian;

    Point o_n;
    o_n.X() = transform[noOfJoints](0, 3);
    o_n.Y() = transform[noOfJoints](1, 3);
    o_n.Z() = transform[noOfJoints](2, 3);

    for (int i = 0; i < noOfJoints; i++)
    {
        Point o_i;
        o_i.X() = transform[i](0, 3);
        o_i.Y() = transform[i](1, 3);
        o_i.Z() = transform[i](2, 3);

        Point z_i;
        if ((i == 0) || ((noOfJoints == 6) && ((i == 3) || (i == 5))))
        {
            z_i.X() = transform[i](0, 2);
            z_i.Y() = transform[i](1, 2);
            z_i.Z() = transform[i](2, 2);
        }
        else
        {
            z_i.X() = transform[i](0, 0);
            z_i.Y() = transform[i](1, 0);
            z_i.Z() = transform[i](2, 0);
        }

        Point d = o_n - o_i;
        Point j_v = z_i.CrossProduct(d);

        float jacobian_i[] = {j_v.X(), j_v.Y(), j_v.Z(), z_i.X(), z_i.Y(), z_i.Z()};

        for (int j = 0; j < noOfJoints; j++)
        {
            jacobian(j, i) = jacobian_i[j];
        }
    }

    return (jacobian);
}

// Full inverse velocity kinematics to calculate speed each motor should rotate at given a direction to move in
Matrix<noOfJoints, 1> RobotArm::InverseVelocityKinematics(Matrix<4, 4> o[], float r[], float t[], Matrix<noOfJoints, 1> targetVelocity)
{
    ForwardKinematics(o, r, t);
    return (GaussianElimination(CalculateJacobian(o), targetVelocity));
}