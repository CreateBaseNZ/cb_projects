#include "Helper.h"

// Transformation matrix for intrinsic rotation about object's X-axis
Matrix<4, 4> Rotate_X(float theta)
{
    Matrix<4, 4> M = {1, 0, 0, 0,
                      0, cos(theta), -sin(theta), 0,
                      0, sin(theta), cos(theta), 0,
                      0, 0, 0, 1};
    return (M);
}

// Transformation matrix for intrinsic rotation about object's Y-axis
Matrix<4, 4> Rotate_Y(float theta)
{
    Matrix<4, 4> M = {cos(theta), 0, sin(theta), 0,
                      0, 1, 0, 0,
                      -sin(theta), 0, cos(theta), 0,
                      0, 0, 0, 1};

    return (M);
}

// Transformation matrix for intrinsic rotation about object's Z-axis
Matrix<4, 4> Rotate_Z(float theta)
{
    Matrix<4, 4> M = {cos(theta), -sin(theta), 0, 0,
                      sin(theta), cos(theta), 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1};

    return (M);
}

// Transformation matrix for translation along a 3D vector
Matrix<4, 4> Translate_XYZ(float x, float y, float z)
{
    Matrix<4, 4> M = {1, 0, 0, x,
                      0, 1, 0, y,
                      0, 0, 1, z,
                      0, 0, 0, 1};

    return (M);
}