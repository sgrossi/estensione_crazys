#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <cmath>
#include <iostream>
#include <math.h>
#include "map_functions.h"

using namespace Eigen;

Vector3d vee(Matrix3d SkewSym) {

    if (abs(SkewSym(0,0)) < 1e-2 || abs(SkewSym(1,1)) < 1e-2 || abs(SkewSym(2,2)) < 1e-2)
    {
    }
    if (abs(SkewSym(1,0)+SkewSym(0,1)) < 1e-2 || abs(SkewSym(2,0)+SkewSym(0,2)) < 1e-2
             || abs(SkewSym(2,1)+SkewSym(1,2)) < 1e-2)
    {
    }
    else
    {
            std::cout << "Matrice non antisimmetrica" << std::endl;
    }
    Vector3d    result(SkewSym(2,1), SkewSym(0,2), SkewSym(1,0));
    return result;
}

Matrix3d hat(Vector3d Vec) {

    Matrix3d result;

    result <<   0,      -Vec(2),     Vec(1),
                Vec(2), 0,           -Vec(0),
                -Vec(1),Vec(0),     0;

    return result;
}
