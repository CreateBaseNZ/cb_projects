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

void RobotArm::AttachMotors(int pins[])
{
    for (int i = 0; i < noOfJoints; i++)
    {
        servoMotors[i].attach(pins[i]);
    }
}

void RobotArm::DetachMotors()
{
    for (int i = 0; i < noOfJoints; i++)
    {
        servoMotors[i].detach();
    }
}
void RobotArm::ConfigureBasketBall(int pins[],int number,int limit[]){
    for(int i=0;i<number;i++){
        pinMode(pins[i],INPUT);
        limits[i]=limit[i];
        phototransisorPins[i]=pins[i];
    }
    noOfTransitors=number;
}

void RobotArm::ConfigurePins(int pins[])
{
    for (int i = 0; i < noOfJoints; i++){
        servoMotors[i].attach(pins[i]);
        servoMotors[i].write(90);
    }
}
void RobotArm::ConfigureUltraSonic(int pins[],int sonars){
    for(int i=0;i<sonars;i++){
        sonarTrigPin[i]=pins[i];
        pinMode(sonarTrigPin[i],OUTPUT);
        pinMode(sonarTrigPin[i]+1,INPUT);
    }
    sonarUsed=sonars;

}

float RobotArm::findDistance(int sonarNo){
    if( sonarNo>=0 && sonarNo<sonarUsed){
        int sonarPin=sonarTrigPin[sonarNo];
        digitalWrite(sonarPin, HIGH);
        delayMicroseconds(2);
        digitalWrite(sonarPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(sonarPin, LOW);
        long duration = pulseIn(sonarPin+1, HIGH,100000);
        if (duration==0){
            return -2;
        }
        return 330*duration/2/10000;
    }
    return -1;
}


float RobotArm::torad(float angle){
    return angle*M_PI/180;
}

float RobotArm::todeg(float angle){
    return angle*180/M_PI;
}



void RobotArm::HandControl(){
    float lowest=10,distance,velocities[3]={0,0,0};int lowsetindex=-1;
    for(int i=0;i<sonarUsed;i++){
        distance=findDistance(i);
        // Serial<<distance<<", ";
        if(distance<lowest&&distance>=0){
            lowsetindex=i;
            lowest=distance;
        }
    }
    // Serial<<"\n";
    // Serial<<lowest<<"\n";

    if(lowsetindex!=-1){    
        velocities[lowsetindex]=0.0025;
        float v_y_dash=0;//(velocities[1]+velocities[2]) *cos(torad(60))-velocities[0];//
        float v_x=(velocities[1]-velocities[2]);//*cos(torad(30));
        float theta=0;
        for(int i=1;i<noOfJoints;i++){
            if(i==1){
                theta+=torad(servoMotors[i].read());
            }else{
                theta+=torad(servoMotors[i].read()-90);
            }
        }
        float v_z=v_y_dash*cos(theta); 
        float v_y=v_y_dash*sin(theta);     

        Move(v_x,v_y,v_z,0,0,0);

    }
    else{
        for(int j=0;j<noOfJoints;j++){
                servoMotors[j].stop();
        }
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


float RobotArm::roundXdp(float input,int decimalPlaces){
    float divider=pow(10,-decimalPlaces);
    return (round(input/divider))*divider;
}



bool RobotArm::Move_position_4link(float r,float z, float theta_deg, float alpha_deg){
    float pi=M_PI,pi_2=M_PI_2;
    float alpha=Circle_round(torad(alpha_deg));
    float theta=torad(theta_deg);
    float R,C,angles[4],cosAngle_2=0.0,cosAngle_2_num; 

    angles[0]=atan(sin(alpha)/ cos(alpha));
    R=-linkLengths[3]*cos(theta);
    if(abs(alpha)>pi_2){
        R-=r;
    }else{
        R+=r;
    }
    if(abs(R)<0.001){
        R=0;
    }
    C=z-linkLengths[0]-linkLengths[3]*sin(theta);
    cosAngle_2_num=pow(R,2)+pow(C,2)-pow(linkLengths[1],2)-pow(linkLengths[2],2);
    if(abs(cosAngle_2_num)<0.001){
        angles[2]=-pi/2;
    }else if(abs(pow(R,2)+pow(C,2)-pow((linkLengths[1]+linkLengths[2]),2))<0.0001){
        angles[2]=0;
    }
    else{
        cosAngle_2=cosAngle_2_num/(2*linkLengths[2]*linkLengths[1]);
        if(cosAngle_2>1||cosAngle_2<0){
            Serial<<"Can't Reach Point.\n";
            return false;
        }
        angles[2]=acos(cosAngle_2);
    }

    if(abs(C)<0.0001){
        C=0;
    }
    findAngles1_3(angles,theta,R,C);
    if(abs(angles[3])>pi_2||angles[1]<0||angles[1]>pi){
        angles[2]=-angles[2];
        findAngles1_3(angles,theta,R,C);
    }
    for(int i=0;i<noOfJoints;i++){
        angles[i]=roundXdp(angles[i],2);
        if(i==1){
            if (angles[i]>M_PI ||angles[i]<0){
                Serial<<"Cant Reach Point.\n";
                return false;
            }
        }else{
            if (abs(angles[i])>M_PI_2){
                Serial<<"Cant Reach Point.\n";
                return false;
            }
        }
    }

    for( int i=0;i<noOfJoints;i++){
        if(i!=1){
            angles[i]+=M_PI_2;
        }
        angles[i]=round(todeg(angles[i]));
        servoMotors[i].write(angles[i]);
        Serial<<angles[i]<<", ";
    }
    Serial<<"\n";
    return true;
}

void RobotArm::findAngles1_3(float angles[],float theta,float R,float C){
    float a,b,pi=M_PI;
    a=roundXdp(linkLengths[2]*sin(angles[2]),4);
    b=roundXdp(linkLengths[1]+linkLengths[2]*cos(angles[2]),4);
    float At_1,At_2;
    At_2=atan2(a,b);
    At_1=atan2(C,R);                    
    angles[1]=roundXdp(At_1-At_2,3);
    if(angles[1]<-0.01){
        angles[1]+=2*pi;
    }
    angles[3]=theta-angles[2]-angles[1];
    angles[3]=Circle_round(angles[3]);
}


bool RobotArm::Move_position_xyz(float x, float y,float z, float theta_deg){
    float pi=M_PI,pi_2=M_PI_2;
    float theta=torad(theta_deg);
    float A,B,C,angles[4],cosAngle_2=0.0,cosAngle_2_num; 

    angles[0]=atan(y/ x);
    A = (x - linkLengths[3] * cos(angles[0]) * cos(theta));
    B = (y - linkLengths[3] * sin(angles[0]) * cos(theta));
    C = z - linkLengths[0] - linkLengths[3] * sin(theta);
    cosAngle_2_num = pow(A, 2) + pow(B, 2) + pow(C, 2) - pow(linkLengths[1], 2) - pow(linkLengths[2], 2);
    if (abs(cosAngle_2_num) < 0.001)
    {
        angles[2]=-pi/2;
    }
    else if (abs(pow(A, 2) + pow(B, 2) + pow(C, 2) - pow((linkLengths[1] + linkLengths[2]), 2)) < 0.0001)
    {
        angles[2]=0;
    }
    else{
        cosAngle_2=cosAngle_2_num/(2*linkLengths[2]*linkLengths[1]);
        if(cosAngle_2>1||cosAngle_2<0){
            Serial<<"Can't Reach Point.\n";
            return false;
        }
        angles[2]=acos(cosAngle_2);
    }

    if(abs(C)<0.0001){
        C=0;
    }
    float R;
    R = pow(A, 2) + pow(B, 2);
    if(A<-0.001){
        R = -R;
    }
    findAngles1_3(angles, theta, R, C);
    if(abs(angles[3])>pi_2||angles[1]<0||angles[1]>pi){
        angles[2]=-angles[2];
        findAngles1_3(angles,theta,R,C);
    }
    for(int i=0;i<noOfJoints;i++){
        angles[i]=roundXdp(angles[i],2);
        if(i==1){
            if (angles[i]>M_PI ||angles[i]<0){
                Serial<<"Cant Reach Point.\n";
                return false;
            }
        }else{
            if (abs(angles[i])>M_PI_2){
                Serial<<"Cant Reach Point.\n";
                return false;
            }
        }
    }

    for( int i=0;i<noOfJoints;i++){
        if(i!=1){
            angles[i]+=M_PI_2;
        }
        angles[i]=round(todeg(angles[i]));
        servoMotors[i].write(angles[i]);
        Serial<<angles[i]<<", ";
    }
    Serial<<"\n";
    return true;
}



// Uses velocity IK to determine actuation for motors
void RobotArm::Move(float vx, float vy, float vz, float wx, float wy, float wz)
{      

    Matrix<4,1> targetVel={vx,vy,vz,wx};

    for(int i=0;i<noOfJoints;i++){
        jointAngles[i]=Mapf(servoMotors[i].read(),0,180,-M_PI_2,M_PI_2);

    }  

    Matrix<noOfJoints,1> servoVelocity=InverseVelocityKinematics(jointAngles,linkLengths,targetVel);
    for (int i=0;i<noOfJoints;i++){
        int dir=0;
        if(servoVelocity(i)>0){
            dir=180;
        }
        if((servoMotors[i].read()==0&&servoVelocity(i)<0)||(servoMotors[i].read()==180&&servoVelocity(i)>0||isnan(servoVelocity(i)))){
            for(int j=0;j<noOfJoints;j++){
                servoMotors[j].write(Mapf(jointAngles[j],-M_PI_2,M_PI_2,0,180));
            }
            break;
        }
        float speed_ticks=(servoVelocity(i)*180.0*2.5/M_PI);
        speed_ticks=(abs(speed_ticks));
        if(speed_ticks<=1){
            servoMotors[i].write(Mapf(jointAngles[i],-M_PI_2,M_PI_2,0,180));
        }else{
            servoMotors[i].write(dir,speed_ticks+1);
        }
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

    for (int i = 0; i < noOfJoints; i++)
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
            if(i==0&&j==3){
                jacobian(j,i)=0;
            }
        }
    }
    return (jacobian);
}

// Full inverse velocity kinematics to calculate speed each motor should rotate at given a direction to move in
Matrix<noOfJoints, 1> RobotArm::InverseVelocityKinematics( float r[], float t[], Matrix<noOfJoints, 1> targetVelocity)
{
    Matrix<4, 4> o[noOfJoints+1];
    ForwardKinematics(o, r, t);
    Matrix<noOfJoints,noOfJoints> VelForwardKinematics=CalculateJacobian(o);
    if(VelForwardKinematics.Det()!=0){
        Matrix<noOfJoints,noOfJoints> VelInverseKinematics=VelForwardKinematics.Inverse();
        Matrix<noOfJoints, 1> k=(VelInverseKinematics*targetVelocity);
        
        return k;
    }else{
        Matrix<noOfJoints,1> zeroVector = {0, 0, 0, 0};
        return zeroVector;
    }
}

bool RobotArm::DetectPassage(){
   
    for (int i=0;i<3;i++){
        int val=analogRead(phototransisorPins[i]);
        Serial<<val<<",";
        if(val>limits[i]){
            Serial<<"Score!!"<<"\n";
           delay(1000);
           return true;
        }
    }
    //delay(250);
    Serial<<"\n";
    return false;
    //  
    // 
    // if(val<350){
    //     return false;
    // }else{
    //     Serial<<"Score!!"<<"\n";
    //     delay(1000);
    //     return true;
    // }
}






//////////////////////////////////////////////////////

void RobotArm::ResetDraw(){
    DrawValue=0;
    DrawingDone=false;
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

        if(Move_position_4link(raduis,z,drawingAngle,angle)){
            bool notReached=false;
            for(int i=0;i<noOfJoints && !notReached;i++){
                float error=servoMotors[i].read()-GetServoDegrees(i);
                if(abs(error)>10){
                    notReached=true;
                }
            }
            if(!notReached){
                if(DrawValue*interval>=360){
                    DrawingDone=true;
                    return;
                }else{
                    DrawValue++;
                }
            }
        }else{
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
                if(abs(error)>5){
                    notReached=true;
                }
            }
            if(!notReached){
                if(DrawValue*interval>=360){
                    DrawingDone=true;
                    return;
                }else{
                    DrawValue++;
                }
            }
        }else{
            DrawingDone=true;
            return;
        }
    }
}

void RobotArm::FindLocation(float locations[]){
    float servoAngles[noOfJoints];
    for(int i=0;i<noOfJoints;i++){
        if(i==1){
            servoAngles[i]=torad(servoMotors[i].read());
        }else{
            servoAngles[i]=torad(servoMotors[i].read()-90);
        }
    }    
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

// Calibration sequence takes the servos to their limits and records
// the positional feedback value
void RobotArm::CalibrateServos()
{
    Serial.println("Calibrating...");
    for (int i = 0; i < noOfJoints; i++)
    {
        Serial << "Calirbrating Lower: " << i << "\n";
        servoMotors[i].write(0);
        delay(3000);        
        servoLowerLimit[i] = analogRead(i);
        servoMotors[i].write(90);
    }
    delay(3000);
    for (int i = 0; i < noOfJoints; i++)
    {
        Serial << "Calirbrating higher: " << i << "\n";
        servoMotors[i].write(180);
        delay(3000);
        servoUpperLimit[i] = analogRead(i);
        servoMotors[i].write(90);
    }
    Serial.println("Calibration Complete");
}
