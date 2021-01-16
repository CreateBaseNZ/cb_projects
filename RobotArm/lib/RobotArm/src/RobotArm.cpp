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
void RobotArm::ConfigureUltraSonic(int pins[noOfSonar]){
    for(int i=0;i<noOfSonar;i++){
        sonarTrigPin[i]=pins[i];
        pinMode(sonarTrigPin[i],OUTPUT);
        pinMode(sonarTrigPin[i]+1,INPUT);
    }
    Ultrasonic=true;
}

float RobotArm::findDistance(int sonarNo){
    if(Ultrasonic){
        int sonarPin=sonarTrigPin[sonarNo];
        digitalWrite(sonarPin, HIGH);
        delayMicroseconds(2);
        digitalWrite(sonarPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(sonarPin, LOW);
        long duration = pulseIn(sonarPin+1, HIGH);
        if (duration==0){
            return -2;
        }
        return 330*duration/2/10004;
    }
    return -1;
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


void RobotArm::HandControl(float time){
    float lowest=40,distance,velocities[noOfSonar]={0,0,0};int lowsetindex=-1;
    Serial<<"distancesss ";
    for(int i=0;i<noOfSonar;i++){
        distance=findDistance(i);
        if(distance<lowest&&distance>=0){
            lowsetindex=i;
            lowest=distance;
        }
        Serial<<distance<<", ";
    }
    Serial<<"\n";
    Serial<<lowsetindex<<"\n";
    if(lowsetindex!=-1){    
        velocities[lowsetindex]=0.01;

        float position[4];
        FindLocation(position);
        float v_y=(velocities[1]+velocities[2])*cos(torad(60))- velocities[0];
        float v_x=-(velocities[1]-velocities[2])*cos(torad(30));
        Serial<<" vel:(x,y)"<<v_x<<", "<<v_y<<"\n";
        float theta=torad(position[2]),r=position[0],alpha=torad(position[3]);
        float v_z=v_y*cos(theta); 
        float v_alpha,v_r;
        if(cos(alpha)==0){
            v_alpha=0;
            v_r=v_y/sin(alpha);
        }
        else{
            v_alpha=v_x/(r*cos(alpha)+r*(pow(sin((alpha)),2))/cos((alpha)));
            v_r=r*v_alpha*sin((alpha))/cos(alpha);
        }
        v_r-=v_y*sin(theta);
        
        Serial<<"Velocity(v_z,v_r,v_alpha): "<<v_z<< ", "<<v_r<<", "<<v_alpha<<"\n";
        Move(v_r,v_z,0,todeg(v_alpha),0,0,time);
    } 
}


float RobotArm::Circle_round(float input){
    while(input<=-M_PI){
        input+=2*M_PI;
    }
    while(input>M_PI){
        input-=2*M_PI;
    }
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
float RobotArm::roundXdp(float input,int decimalPlaces){
    float divider=pow(10,-decimalPlaces);
    return (round(input/divider))*divider;
}



bool RobotArm::Move_position_4link(float r,float z, float theta_deg, float alpha_deg){
    float pi=M_PI,pi_2=M_PI_2;
    float alpha=alpha_deg*M_PI/180;
    float theta=theta_deg*M_PI/180;
    float A,B,C,a,b,angles[4],cosAngle_2=0.0,cosAngle_2_num; 

    alpha=Circle_round(alpha);
    angles[0]=atan(sin(alpha)/ cos(alpha));
    A=r*cos(alpha)-linkLengths[3]*cos(theta)*cos(angles[0]);
    B=r*sin(alpha)-linkLengths[3]*cos(theta)*sin(angles[0]);    
    C=z-linkLengths[0]-linkLengths[3]*sin(theta);
    cosAngle_2_num=pow(A,2)+pow(B,2)+pow(C,2)-pow(linkLengths[1],2)-pow(linkLengths[2],2);
    if(abs(cosAngle_2_num)<0.001){
        angles[2]=pi/2;
    }else if(abs(pow(A,2)+pow(B,2)+pow(C,2)-pow(0.21,2))<0.0001){
        angles[2]=0;
    }
    else{
        cosAngle_2=cosAngle_2_num/(2*linkLengths[2]*linkLengths[1]);
        if(abs(cosAngle_2)>1){
            float g=(pow(A,2)+pow(B,2)+pow(C,2));
            Serial<<"Can't Reach Point: "<<g<<"\n";
            return false;
        }
        angles[2]=acos(cosAngle_2);
    }
    if(abs(C)<0.0001){
        C=0;
    }
    a=roundXdp(linkLengths[2]*sin(angles[2]),4);
    b=roundXdp(linkLengths[1]+linkLengths[2]*cos(angles[2]),4);
    float At_1,At_2;
    At_2=atan2(a,b);
    if(abs(alpha)<=M_PI_2){
        At_1=atan2(C,sqrt(pow(a,2)+pow(b,2)-pow(C,2)));
        Serial<<At_1<<"\n";
        angles[1]=At_1-At_2;
    }else{
        At_1=atan2(C,-sqrt(pow(a,2)+pow(b,2)-pow(C,2)));
        if(At_1<0){
            At_1=At_1+2*pi;
        }
        angles[1]=At_1-At_2;
    }
    angles[3]=theta-angles[2]-angles[1];
    angles[3]=Circle_round(angles[3]);
        Serial<<angles[0]<<", "<<angles[1]<<", "<<angles[2]<<", "<<angles[3]<<"\n";

    if(abs(angles[3])>pi_2||angles[1]<0||angles[1]>pi){
        Serial<<"Why?";
              angles[2]=-angles[2];
        a=roundXdp(linkLengths[2]*sin(angles[2]),4);
        b=roundXdp(linkLengths[1]+linkLengths[2]*cos(angles[2]),4);
        At_2=atan2(a,b);

        if(abs(alpha)<=M_PI_2){
            At_1=atan2(C,sqrt(pow(a,2)+pow(b,2)-pow(C,2)));
            angles[1]=At_1-At_2;
            angles[3]=theta-angles[2]-angles[1];
        }else{
            At_1=atan2(C,-sqrt(pow(a,2)+pow(b,2)-pow(C,2)));
            if(At_1<0){
                At_1=At_1+2*pi;
            }
            angles[1]=At_1-At_2;
            angles[3]=theta-angles[2]-angles[1];
        }
        angles[3]=Circle_round(angles[3]);
    }
    for(int i=0;i<noOfJoints;i++){
        angles[i]=roundXdp(angles[i],2);
        if(i==1){
            if (angles[i]>M_PI ||angles[i]<0){
                 Serial<<"Cant Reach Point 2\n";
                                  Serial<<angles[0]<<", "<<angles[1]<<", "<<angles[2]<<", "<<angles[3]<<"\n";

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
        angles[i]=round(angles[i]*180/M_PI);
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