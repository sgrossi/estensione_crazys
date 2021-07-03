#include "dirtyderivative.h"
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

using namespace Eigen;

dxdt::dxdt(unsigned int ORDER, double TAU, double TS)
{
    order = ORDER;
    tau = TAU;
    Ts = TS;

    it = 1;

    a1 = (2*tau - Ts)/(2*tau + Ts);
    a2 = 2 / (2*tau + Ts);
}

Vector3d dxdt::calculate(Vector3d x)
{
    if (it==1)
    {
        dot << 0,0,0;
        x_d1 << 0,0,0;
    }

    if (it > order)
    {
        // Calcolo derivata
        dot = a1*dot + a2*(x-x_d1);
    }

    it++;
    x_d1 = x;

    return dot;
}
