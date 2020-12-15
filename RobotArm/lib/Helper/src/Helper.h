#ifndef Helper_h
#define Helper_h

#include "Geometry.h"
#include "BasicLinearAlgebra.h"
#include "Arduino.h"

Matrix<4, 4> Rotate_X(float theta);
Matrix<4, 4> Rotate_Y(float theta);
Matrix<4, 4> Rotate_Z(float theta);
Matrix<4, 4> Translate_XYZ(float x, float y, float z);

void ParseCSV(String serialString, float outputData[]);

#endif