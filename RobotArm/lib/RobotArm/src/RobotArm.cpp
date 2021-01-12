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

        servoMotors[i].write(0);
        // if (i == 1)
        // {
        //     servoMotors[i + 1].writeMicroseconds(2500);
        // }
        delay(3000);
        char input ='s';
        
        servoLowerLimit[i] = analogRead(i);
        servoMotors[i].write(90);
    }

    delay(3000);

    for (int i = 0; i < noOfJoints; i++)
    {
        Serial << "Calirbrating higher: " << i << "\n";


        servoMotors[i].write(180);
        // if (i == 1)
        // {
        //     servoMotors[i + 1].writeMicroseconds(500);
        // }
        delay(3000);

        // while(input!='d'){
        //     if(Serial.available()){
        //         input=Serial.read();
        //     }
        // }
        servoUpperLimit[i] = analogRead(i);
        servoMotors[i].write(90);
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
    while(input<-M_PI){
        input+=2*M_PI;
    }
    input=todeg(input);
    input=(int(input+90))%180;
    input=torad(input-90);
    input=round(input/0.01)*0.01;

    return input;
}

float RobotArm::round2dp(float input){
    return round(input/0.01)*0.01;
}

float RobotArm::Circle_round(float input){
    while(input<-M_PI){
        input+=2*M_PI;
    }
    input=todeg(input);
    input=(int(input+180))%360;
    input=torad(input-180);
    input=round(input/0.01)*0.01;

    return input;
}


void RobotArm::DrawSquare(float Length,float z){
    if(!DrawingDone){
        int interval=360/45; 
        float raduis;
        int angle=DrawValue*interval;
        int inputAngle=angle;
        while(inputAngle>45){
            inputAngle-=90;
        }
        raduis=Length/cos(torad(inputAngle));
        int drawingAngle=0;
        if(angle>90&&angle<270){
            drawingAngle=180;
        }

        Serial<<"Point no. "<<DrawValue<<": "<<raduis<<", "<<z<<","<<angle<<"\n";
        if(Move_position_4link(raduis,z,drawingAngle,angle)){
            bool notReached=false;
            for(int i=0;i<noOfJoints && !notReached;i++){
                float error=servoMotors[i].read()-GetServoDegrees(i);
                Serial<<"Motor "<<i<<": "<<servoMotors[i].read()<<", "<<GetServoDegrees(i)<<" with error of "<<abs(error)<<"\n";
                if(abs(error)>10){
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

void RobotArm::DrawCircle(float raduis, float z){
    int interval= 360/45;
    if(!DrawingDone){
        if(Move_position_4link(raduis,z,-90,DrawValue*interval)){
            bool notReached=false;
            for(int i=0;i<noOfJoints && !notReached;i++){
                float error=servoMotors[i].read()-GetServoDegrees(i);
                Serial<<"Motor "<<i<<": "<<servoMotors[i].read()<<", "<<GetServoDegrees(i)<<" with error of "<<abs(error)<<"\n";
                if(abs(error)>5){
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
    float A,B,C,a,b,angles[4],cosAngle_2=0.0; 


    alpha=Circle_round(alpha);
    angles[0]=atan(sin(alpha)/ cos(alpha));
    
    A=r*cos(alpha)-linkLengths[3]*cos(theta)*cos(angles[0]);
    B=r*sin(alpha)-linkLengths[3]*cos(theta)*sin(angles[0]);    
    C=z-linkLengths[0]-linkLengths[3]*sin(theta);
    cosAngle_2=(float)((pow(A,2)+pow(B,2)+pow(C,2)-pow(linkLengths[1],2)-pow(linkLengths[2],2))/(2*linkLengths[2]*linkLengths[1]));
    if(abs(cosAngle_2)>1){
        // Serial<<"Cant Reach Point 1\n";
        return false;
    }
    Serial<<"Cosine: "<<cosAngle_2<<"\n";
    angles[2]=(acos(cosAngle_2));
    Serial<<"Angle_2: "<<angles[2]<<"\n";
    if(abs(alpha)<=M_PI_2){
        angles[2]=-angles[2];
    }
    Serial<<"Angle_2: "<<angles[2]<<"\n";

    a=linkLengths[2]*sin(angles[2]);
    b=linkLengths[1]+linkLengths[2]*cos(angles[2]);
    angles[1]=(atan2(C,sqrt(pow(a,2)+pow(b,2)-pow(C,2)))-atan2(a,b));

    angles[3]=(theta-angles[2]-angles[1]);
                                //  Serial<<angles[0]<<", "<<angles[1]<<", "<<angles[2]<<", "<<angles[3]<<"\n";
            angles[1]=round2dp(angles[1]);
                    angles[3]=Circle_round(angles[3]);


    if(angles[1]>M_PI ||angles[1]<0||abs(angles[3])>M_PI_2){
                                        //  Serial<<angles[0]<<", "<<angles[1]<<", "<<angles[2]<<", "<<angles[3]<<"\n";

        angles[1]=(atan2(C,-sqrt(pow(a,2)+pow(b,2)-pow(C,2)))-atan2(a,b));
        angles[3]=Circle_round(theta-angles[2]-angles[1]);
    }

    for(int i=0;i<noOfJoints;i++){
        angles[i]=round2dp(angles[i]);
        if(i==1){
            if (angles[i]>M_PI ||angles[i]<0){
                 Serial<<"Cant Reach Point 2\n";
                //                  Serial<<angles[0]<<", "<<angles[1]<<", "<<angles[2]<<", "<<angles[3]<<"\n";

                return false;
            }
        }else{
            if (abs(angles[i])>M_PI_2){
                 Serial<<"Cant Reach Point 3: " <<i<<"\n";
                  Serial<<angles[0]<<", "<<angles[1]<<", "<<angles[2]<<", "<<angles[3]<<"\n";

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
    Serial<<angles[0]<<", "<<angles[1]<<", "<<angles[2]<<", "<<angles[3]<<"\n";

    return true;
    
}

// Uses velocity IK to determine actuation for motors
void RobotArm::Move(float r_dot, float z_dot, float theta_dot, float alpha_dot, float wy, float wz,float time)
{
    float targetVelocity[4] = {r_dot, z_dot, theta_dot, alpha_dot};


    float location[4]; 
    FindLocation(location);    
    Serial<<"old: "<<location[0]<<", "<<location[1]<<", "<<(location[2])<<", "<<(location[3])<<", "<<time<<"\n";

    float newLocation[4];
    for(int i=0;i<4;i++){
        newLocation[i]=location[i]+time*targetVelocity[i];
    location[i]=newLocation[i];

    }
    Move_position_4link(abs(newLocation[0]),newLocation[1],newLocation[2],newLocation[3]);
    Serial<<"New: "<<location[0]<<", "<<location[1]<<", "<<(location[2])<<", "<<(location[3])<<"\n";

}

float RobotArm::GetServoDegrees(int servo)
{
    return (Mapf(analogRead(servo), servoLowerLimit[servo], servoUpperLimit[servo], 0, 180));
}

float RobotArm::Mapf(float value, float fromLow, float fromHigh, float toLow, float toHigh)
{
    return ((value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow);
}

void RobotArm::FindLocation(float locations[]){
    float servoAngles[noOfJoints];
    Serial<<"Angles: ";
    for(int i=0;i<noOfJoints;i++){
        if(i==1){
            servoAngles[i]=torad(servoMotors[i].read());
        }else{
            servoAngles[i]=torad(servoMotors[i].read()-90);
        }
        Serial<<servoMotors[i].read()<<", ";
    }    
    Serial<<"\n";
    float x=0,y=0;
    locations[1]=linkLengths[0];
    for(int i=1;i<noOfJoints;i++){
        float totalAngle_T_A_B[]={0,0,0};
        for(int j=i;j>=0;j--){
            if(j>0&&j<4){
                totalAngle_T_A_B[0]+=servoAngles[j];
            }
            else if(j==0){
                totalAngle_T_A_B[1]+=servoAngles[j];
            }
        }            
        x+=linkLengths[i]*cos(totalAngle_T_A_B[0])*cos(totalAngle_T_A_B[1]);
        y+=linkLengths[i]*cos(totalAngle_T_A_B[0])*sin(totalAngle_T_A_B[1]);
        locations[1]+=linkLengths[i]*sin(totalAngle_T_A_B[0]);
        if(i==noOfJoints-1){
            locations[2]=todeg(totalAngle_T_A_B[0]);
        }
    }
    locations[3]=todeg(atan2(y,x));
    locations[0]=abs(sqrt(pow(x,2)+pow(y,2)));
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