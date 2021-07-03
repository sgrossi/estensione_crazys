#ifndef CONVERSIONS
#define CONVERSIONS

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <cmath>
#include <iostream>
#include <math.h>

using namespace Eigen;

Matrix3d QuatToRot(double qi, double qj, double qk, double qr);
Vector4d FMToAngVelocities(double force, Vector3d moments, Matrix4d mix, double maxVel);

#endif
