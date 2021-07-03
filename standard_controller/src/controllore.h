#ifndef CONTROLLORE
#define CONTROLLORE
#include "pid.h"

void ErrorBodyFrame(double xref, double x, double yref, double y, double psi, double& xe, double& ye);
void XYController(double xe, double u, double ye, double v, PID& PIDtheta_c, PID& PIDphi_c, double& theta_c, double& phi_c);
void AttitudeController(double phi_c, double phi, double theta_c, double theta, PID& PIDpc, PID& PIDqc, double& pc, double& qc);
void RateController(double qc, double q, double pc, double p, double rc, double r, PID& PIDdeltatheta, PID& PIDdeltaphi, PID& PIDdeltapsi, double& deltatheta, double& deltaphi, double& deltapsi);
void ControlMixer(double deltatheta, double deltaphi, double deltapsi, double omega, double& pwm1, double& pwm2, double& pwm3, double& pwm4);
void Motor(double pwm1, double pwm2, double pwm3, double pwm4, double motorAngolarCoefficient, double motorsIntercept, double maxRotorVelocity, double& w1, double& w2, double& w3, double& w4);
void YawPositionController(double psi_c, double psi, PID& PIDrc, double& rc);
void HoveringController(double z_c, double z, double vel_w, double Kd_z, double deltaomegaUpBound, double deltaomegaLowBound, PID& PIDdeltaomega, double& deltaomega);

#endif
