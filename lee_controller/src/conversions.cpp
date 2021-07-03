#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <cmath>
#include <iostream>
#include <math.h>
#include "conversions.h"

using namespace Eigen;

Matrix3d QuatToRot(double qi, double qj, double qk, double qr)
{
    double s = pow(qi*qi + qj*qj + qk*qk + qr*qr,-2);

    Matrix3d Rot;
    Rot <<  1-2*s*(qj*qj+qk*qk),    2*s*(qi*qj-qk*qr),      2*s*(qi*qk+qj*qr),
            2*s*(qi*qj+qk*qr),      1-2*s*(qi*qi+qk*qk),    2*s*(qj*qk-qi*qr),
            2*s*(qi*qk-qj*qr),      2*s*(qj*qk+qi*qr),      1-2*s*(qi*qi+qj*qj);

    return Rot;
}

Vector4d FMToAngVelocities(double force, Vector3d moments, Matrix4d mix, double maxVel)
{
    Vector4d inputs(force, moments(0), moments(1), moments(2));

    Vector4d output;
    output = mix * inputs;

    if (output(0) < 0) {
        output(0) = 0;
    }
    else {
        output(0) = sqrt(output(0));
        if (output(0) >= maxVel) {
            output(0) = maxVel;
        }
    }

    if (output(1) < 0) {
        output(1) = 0;
    }
    else {
        output(1) = sqrt(output(1));
        if (output(1) >= maxVel) {
            output(1) = maxVel;
        }
    }

    if (output(2) < 0) {
        output(2) = 0;
    }
    else {
        output(2) = sqrt(output(2));
        if (output(2) >= maxVel) {
            output(2) = maxVel;
        }
    }

    if (output(3) < 0) {
        output(3) = 0;
    }
    else {
        output(3) = sqrt(output(3));
        if (output(3) >= maxVel) {
            output(3) = maxVel;
        }
    }

    return output;
}
