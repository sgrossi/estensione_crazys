#ifndef MAP_FUNCTIONS
#define MAP_FUNCTIONS

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <cmath>
#include <iostream>
#include <math.h>
#include "map_functions.h"

using namespace Eigen;

Vector3d vee(Matrix3d SkewSym);
Matrix3d hat(Vector3d Vec);

#endif
