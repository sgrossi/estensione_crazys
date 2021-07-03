#include <math.h>
#include "controllore.h"
#include "pid.h"

void ErrorBodyFrame(double xref, double x, double yref, double y, double psi, double &xe, double &ye) {
    xe = (xref - x) * cos(psi) + (yref - y) * sin(psi);
    ye = - (xref - x) * sin(psi) + (yref - y) * cos(psi);
}

void XYController(double xe, double u, double ye, double v, PID &PIDtheta_c, PID &PIDphi_c, double &theta_c, double &phi_c) {
    theta_c = PIDtheta_c.calculate(xe, u);
    phi_c = PIDphi_c.calculate(ye, v);
}

void AttitudeController(double phi_c, double phi, double theta_c, double theta, PID &PIDpc, PID &PIDqc, double &pc, double &qc) {
    qc = PIDqc.calculate(theta_c, theta);
    pc = PIDpc.calculate(phi_c, phi);
}

void RateController(double qc, double q, double pc, double p, double rc, double r, PID &PIDdeltatheta, PID &PIDdeltaphi, PID &PIDdeltapsi, double &deltatheta, double &deltaphi, double &deltapsi) {
    deltatheta = PIDdeltatheta.calculate(qc, q);
    deltaphi = PIDdeltaphi.calculate(pc, p);
    deltapsi = PIDdeltapsi.calculate(rc, r);
}

void ControlMixer(double deltatheta, double deltaphi, double deltapsi, double omega, double &pwm1, double &pwm2, double &pwm3, double &pwm4) {
    pwm1 = omega - deltatheta/2 - deltaphi/2 - deltapsi;
    pwm2 = omega + deltatheta/2 - deltaphi/2 + deltapsi;
    pwm3 = omega + deltatheta/2 + deltaphi/2 - deltapsi;
    pwm4 = omega - deltatheta/2 + deltaphi/2 + deltapsi;
}

void Motor(double pwm1, double pwm2, double pwm3, double pwm4, double motorAngolarCoefficient, double motorsIntercept, double maxRotorVelocity, double &w1, double &w2, double &w3, double &w4) {
    w1 = pwm1 * motorAngolarCoefficient + motorsIntercept;
    w2 = pwm2 * motorAngolarCoefficient + motorsIntercept;
    w3 = pwm3 * motorAngolarCoefficient + motorsIntercept;
    w4 = pwm4 * motorAngolarCoefficient + motorsIntercept;
    if (w1 > maxRotorVelocity) { w1 = maxRotorVelocity; }
    if (w2 > maxRotorVelocity) { w2 = maxRotorVelocity; }
    if (w3 > maxRotorVelocity) { w3 = maxRotorVelocity; }
    if (w4 > maxRotorVelocity) { w4 = maxRotorVelocity; }
}

void YawPositionController(double psi_c, double psi, PID &PIDrc, double &rc) {
    rc = PIDrc.calculate(psi_c, psi);
}

void HoveringController(double z_c, double z, double vel_w, double Kd_z, double deltaomegaUpBound, double deltaomegaLowBound, PID &PIDdeltaomega, double &deltaomega) {
        deltaomega = PIDdeltaomega.calculate(z_c,z);
        deltaomega = deltaomega + Kd_z * vel_w;
        if (deltaomega > deltaomegaUpBound) {deltaomega = deltaomegaUpBound;}
        if (deltaomega < deltaomegaLowBound) {deltaomega = deltaomegaLowBound;}
}
