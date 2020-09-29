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

void RobotArm::ConfigurePins()
{
    for (int i = 0; i < noOfJoints; i++)
    {
        Serial << "Configuring Pin: " << i << "\n";
        servoMotors[i].attach(i + 2, 500, 2500);
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

void RobotArm::CalibrateServos()
{
    Serial.println("Calibrating...");
    for (int i = 0; i < noOfJoints; i++)
    {
        servoMotors[i].writeMicroseconds(500);
        delay(3000);
        servoLowerLimit[i] = analogRead(i);
        servoMotors[i].writeMicroseconds(1500);
    }

    delay(3000);

    for (int i = 0; i < noOfJoints; i++)
    {
        servoMotors[i].writeMicroseconds(2500);
        delay(3000);
        servoUpperLimit[i] = analogRead(i);
        servoMotors[i].writeMicroseconds(1500);
    }

    Serial.println("Calibration Complete");
}

void RobotArm::Move(Direction inputDirection)
{
    float speed = 2;
    Matrix<4, 1> targetVelocity = {0, 0, 0, 0};

    switch (inputDirection)
    {
    case Up:
        targetVelocity(2) = 1 * speed;
        break;
    case Down:
        targetVelocity(2) = -1 * speed;
        break;
    case Left:
        targetVelocity(1) = 1 * speed;
        break;
    case Right:
        targetVelocity(1) = -1 * speed;
        break;
    case Forward:
        targetVelocity(0) = 1 * speed;
        break;
    case Backward:
        targetVelocity(0) = -1 * speed;
        break;
    case PitchUp:
        targetVelocity(3) = 5 * speed;
        break;
    case PitchDown:
        targetVelocity(3) = -5 * speed;
        break;
    default:
        break;
    }

    for (int i = 0; i < noOfJoints; i++)
    {
        jointAngles[i] = GetServoDegrees(i);
        if (i > 0)
        {
            jointAngles[i] -= 90;
        }
        jointAngles[i] = Mapf(jointAngles[i], -90, 90, -M_PI_2, M_PI_2);
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
    return(Mapf(analogRead(servo), servoLowerLimit[servo], servoUpperLimit[servo], 0, 180));
}

float RobotArm::Mapf(float value, float fromLow, float fromHigh, float toLow, float toHigh)
{
    return ((value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow);
}

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

Matrix<noOfJoints, 1> RobotArm::GaussianElimination(Matrix<noOfJoints, noOfJoints> jacobian, Matrix<noOfJoints, 1> targetVelocity)
{
    Matrix<noOfJoints, noOfJoints + 1> augment = jacobian || targetVelocity;
    float scalar;

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

    Matrix<noOfJoints, 1> jointVelocity;
    for (int i = 0; i < noOfJoints; i++)
    {
        jointVelocity(i) = augment(i, noOfJoints);
    }

    return jointVelocity;
}

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

Matrix<noOfJoints, 1> RobotArm::InverseVelocityKinematics(Matrix<4, 4> o[], float r[], float t[], Matrix<noOfJoints, 1> targetVelocity)
{
    ForwardKinematics(o, r, t);
    return (GaussianElimination(CalculateJacobian(o), targetVelocity));
}