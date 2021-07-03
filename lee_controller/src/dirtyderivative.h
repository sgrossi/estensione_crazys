#ifndef DIRTY_DERIVATIVE_H
#define DIRTY_DERIVATIVE_H

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

using namespace Eigen;

class dxdt
{
public:
    dxdt(unsigned int ORDER, double TAU, double TS);
    Vector3d calculate(Vector3d x);

protected:
    double tau;
    double Ts;
    double a1;
    double a2;
    unsigned int order;
    Vector3d dot;
    Vector3d x_d1;
    unsigned long int it;
};

#endif
