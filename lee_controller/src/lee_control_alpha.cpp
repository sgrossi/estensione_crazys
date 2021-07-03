#include <iostream>

#include <ros/ros.h>

#include <mav_msgs/default_topics.h>
#include <ros/console.h>
#include <time.h>
#include <chrono>

#include <stdio.h>
#include <math.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/DroneState.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <sensor_msgs/Imu.h>
#include <ros/callback_queue.h>

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include "map_functions.h"
#include "dirtyderivative.h"
#include "conversions.h"

// PARAMETRI

using namespace Eigen;

// Parametri generali del drone
double Ts = 0.01;

double  gravity = 9.81;
double  mass = 0.027;

double  Jxx = 1.657171e-05;
double  Jyy = 1.657171e-05;
double  Jzz = 2.9261652e-05;

double  d = 0.046;

double  maxRotorsVelocity = 3052;
double  c_motor = 1.71465181e-08;
double  c_tauf = 0.004459273;

double  tau = 0.05;

double  kx = 4 * mass;
double  kv = 3.6 * mass;
double  kR = 8.81 / 1500; //5000
double  kOmega = 2.54 / 1500; //5000

// Inizializzazione delle Dirty Derivatives

dxdt dx1dt(1, tau, Ts);
dxdt dx2dt(2, tau*2, Ts);
dxdt dx3dt(3, tau*2, Ts);
dxdt dx4dt(4, tau*2, Ts);

dxdt db1dt(1, tau, Ts);
dxdt db2dt(2, tau*2, Ts);

dxdt dv1dt(1, tau, Ts);
dxdt dv2dt(2, tau*2, Ts);

// Variabili Flag
bool    FirstTargetReceived = false;
bool    FirstLoop = true;
// Header per Msg
std_msgs::Header act_header;

// Variabili Odometria
double x_GAZEBO, y_GAZEBO, z_GAZEBO; // Posizione
double x_prev, y_prev, z_prev; // Posizione precedente per computazione velocità
double qi_GAZEBO, qj_GAZEBO, qk_GAZEBO, qr_GAZEBO; // x,y,z,w  Quaternione, orientamento del robot
double p_GAZEBO, q_GAZEBO, r_GAZEBO; // Velocità angolari nel frame ABC
// Riferimento
double x_ref, y_ref, z_ref = 0.0;
double x_b1d, y_b1d, z_b1d = 0.0;

// ----------------------------------------------------------------------------------
// FUNZIONI CALLBACK
void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
    x_GAZEBO = odometry_msg->pose.pose.position.x;
    y_GAZEBO = - odometry_msg->pose.pose.position.y;
    z_GAZEBO = - odometry_msg->pose.pose.position.z;

    qi_GAZEBO = odometry_msg->pose.pose.orientation.x;
    qj_GAZEBO = - odometry_msg->pose.pose.orientation.y;
    qk_GAZEBO = - odometry_msg->pose.pose.orientation.z;
    qr_GAZEBO = odometry_msg->pose.pose.orientation.w;

    p_GAZEBO = odometry_msg->twist.twist.angular.x;
    q_GAZEBO = - odometry_msg->twist.twist.angular.y;
    r_GAZEBO = - odometry_msg->twist.twist.angular.z;

    act_header.stamp = odometry_msg->header.stamp;

    // ROS_INFO("Odometria: %f, %f, %f", x_GAZEBO, y_GAZEBO, z_GAZEBO);

}

void ReferenceCallback(const geometry_msgs::Pose::ConstPtr& point_msg)
{
    if (!FirstTargetReceived) {FirstTargetReceived = true;}

    x_ref = point_msg->position.x;
    y_ref = - point_msg->position.y;
    z_ref = - point_msg->position.z;

    x_b1d = point_msg->orientation.x;
    y_b1d = - point_msg->orientation.y;
    z_b1d = - point_msg->orientation.z;

    // ROS_INFO ("Messaggio di target %f %f %f", x_ref, y_ref, z_ref);
}

// ----------------------------------------------------------------------------------
// FUNZIONE MAIN
int main (int argc, char **argv)
{
    ros::init(argc, argv, "controller_lee");

    ros::NodeHandle n;

    ros::Subscriber sub1 = n.subscribe("odometry", 1, OdometryCallback);
    ros::Subscriber sub2 = n.subscribe("command/trajectory", 1, ReferenceCallback);
    ros::Publisher publish_actuators = n.advertise<mav_msgs::Actuators>("command/motor_speed", 1);

    ros::Rate loop_rate(1/Ts);

    Matrix4d Premix;
    Premix <<   c_motor,            c_motor,            c_motor,            c_motor,
                -c_motor*d/sqrt(2), -c_motor*d/sqrt(2), c_motor*d/sqrt(2),  c_motor*d/sqrt(2),
                c_motor*d/sqrt(2),  -c_motor*d/sqrt(2), -c_motor*d/sqrt(2), c_motor*d/sqrt(2),
                c_tauf,             -c_tauf,            c_tauf,             -c_tauf;
    Matrix4d Mix;
    Mix = Premix.inverse();

    Matrix3d J;
    J <<    Jxx,    0,      0,
            0,      Jyy,    0,
            0,      0,      Jzz;

    while (!FirstTargetReceived) {
        loop_rate.sleep();
        ros::spinOnce();

    }

    while (ros::ok())
    {
        // -------------------------------------------------------------------------------
        // SEZIONE ALGORITMO
        // Input
        Vector3d    xd, b1d;

        xd << x_ref, y_ref, z_ref;

        b1d << x_b1d, y_b1d, z_b1d;

        // Stato corrente
        Vector3d    x, v, Omega;
        Matrix3d    R;

        x << x_GAZEBO, y_GAZEBO, z_GAZEBO;

        if (FirstLoop)
        {
            FirstLoop = false;
            v << 0,0,0;
        }
        else
        {
            v(0) = (x_GAZEBO - x_prev)/Ts;
            v(1) = (y_GAZEBO - y_prev)/Ts;
            v(2) = (z_GAZEBO - z_prev)/Ts;
        }
        x_prev = x_GAZEBO;
        y_prev = y_GAZEBO;
        z_prev = z_GAZEBO;

        R = QuatToRot(qi_GAZEBO, qj_GAZEBO, qk_GAZEBO, qr_GAZEBO);

        Omega << p_GAZEBO, q_GAZEBO, r_GAZEBO;

        // Dirty derivatives
        Vector3d    xd_1dot, xd_2dot, xd_3dot, xd_4dot;
        Vector3d    b1d_1dot, b1d_2dot;
        Vector3d    v_1dot, v_2dot;

        xd_1dot = dx1dt.calculate(xd);
        xd_2dot = dx2dt.calculate(xd_1dot);
        xd_3dot = dx3dt.calculate(xd_2dot);
        xd_4dot = dx4dt.calculate(xd_3dot);

        b1d_1dot = db1dt.calculate(b1d);
        b1d_2dot = db2dt.calculate(b1d_1dot);

        v_1dot = dv1dt.calculate(v);
        v_2dot = dv2dt.calculate(v_1dot);

        // Errori base
        Vector3d    ex, ev, ea, ej;
        ex = x - xd;
        ev = v - xd_1dot;
        ea = v_1dot - xd_2dot;
        ej = v_2dot - xd_3dot;

        // Vettore z inerziale
        Vector3d e3(0,0,1);

        // Cacolo spinta totale
        Vector3d A;
        double   f;
        A = -(kx * ex) -(kv * ev) -(mass * gravity * e3) +(mass * xd_2dot);
        f = -A.dot(R*e3);

        // Costruzione orientamento desiderato
        Vector3d    b1c, b2c, b3c, C;
        Matrix3d    Rc;

        b3c = -A / A.norm();
        C = b3c.cross(b1d);
        b2c = C / C.norm();
        b1c = -(1/C.norm()) * b3c.cross(C);

        Rc << b1c(0), b2c(0), b3c(0),
              b1c(1), b2c(1), b3c(1),
              b1c(2), b2c(2), b3c(2);

        // Derivate temporali delle grandezze in body axes
        Vector3d    A_1dot, b3c_1dot, C_1dot, b2c_1dot, b1c_1dot;

        A_1dot   = -kx * ev - kv * ea + mass*xd_3dot;
        b3c_1dot = -A_1dot/A.norm() + (A.dot(A_1dot)/pow(A.norm(),3)) *A;
        C_1dot   = b3c_1dot.cross(b1d) + b3c.cross(b1d_1dot);
        b2c_1dot = C/C.norm() - (C.dot(C_1dot)/pow(C.norm(), 3)) *C;
        b1c_1dot = b2c_1dot.cross(b3c) + b2c.cross(b3c_1dot);

        // Derivate temporali di secondo grado delle grandezze in body axes
        Vector3d    A_2dot, b3c_2dot, C_2dot, b2c_2dot, b1c_2dot;

        A_2dot   = -kx * ea - kv * ej + mass * xd_4dot;
        b3c_2dot = -A_2dot/A.norm() + (2/pow(A.norm(),3))*A.dot(A_1dot)*A_1dot
                 + ((pow(A_1dot.norm(),2) + A.dot(A_2dot))/pow(A.norm(),3))*A
                 - (3/pow(A.norm(),5))*(pow(A.dot(A_1dot),2))*A;
        C_2dot   = b3c_2dot.cross(b1d) + b3c.cross(b1d_2dot)
                   + 2* b3c_1dot.cross(b1d_1dot);
        b2c_2dot = C_2dot/C.norm() - (2/pow(C.norm(),3))*C.dot(C_1dot)*C_1dot
                 - ((pow(C_2dot.norm(),2) + C.dot(C_2dot))/pow(C.norm(),3))*C
                 + (3/pow(C.norm(),5))*(pow(C.dot(C_1dot),2))*C;
        b1c_2dot = b2c_2dot.cross(b3c) + b2c.cross(b3c_2dot)
                 + 2*b2c_1dot.cross(b3c_1dot);

        // Estrarre i valori calcolati di derivate e inserirli in matrici
        Matrix3d    Rc_1dot, Rc_2dot;
        Vector3d    Omegac, Omegac_1dot;

        Rc_1dot <<  b1c_1dot(0), b2c_1dot(0), b3c_1dot(0),
                    b1c_1dot(1), b2c_1dot(1), b3c_1dot(1),
                    b1c_1dot(2), b2c_1dot(2), b3c_1dot(2);

        Rc_2dot <<  b1c_2dot(0), b2c_2dot(0), b3c_2dot(0),
                    b1c_2dot(1), b2c_2dot(1), b3c_2dot(1),
                    b1c_2dot(2), b2c_2dot(2), b3c_2dot(2);

        Omegac  = vee(Rc.transpose() * Rc_1dot);
        Omegac_1dot = vee(Rc.transpose() * Rc_2dot - hat(Omegac)*hat(Omegac));

        // Errori Orientamento
        Vector3d eR, eOmega;
        eR = 0.5 * vee(Rc.transpose()*R - R.transpose()*Rc);
        eOmega = Omega - R.transpose()*Rc*Omegac;

        // Momenti delle forze da applicare
        Vector3d M;
        M = - kR * eR - kOmega * eOmega + Omega.cross(J*Omega)
          - J * (hat(Omega)*R.transpose()*Rc*Omegac - R.transpose()*Rc*Omegac_1dot);

        // --------------------------------------------------------------------------------
        // SEZIONE MAPPATURA VEL. ANGOLARI
        Vector4d ref_rotor_velocities;
        ref_rotor_velocities = FMToAngVelocities(f, M, Mix, maxRotorsVelocity);

        // ROS_INFO ("Pubblico [%f] [%f] [%f] [%f]", ref_rotor_velocities(0), ref_rotor_velocities(1), ref_rotor_velocities(2), ref_rotor_velocities(3));

        // --------------------------------------------------------------------------------
        // SEZIONE PUBBLICAZIONE
        // Creo un nuovo mav_msgs/Actuators per mandare le velocità ai rotori
        mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
        // Clear per essere sicuro che lo spazio di memoria sia pulito
        actuator_msg->angular_velocities.clear();
        // Mettiamo tutte le componenti dell'Eigen::Vector4d nel messaggio da inviare
        for (int i = 0; i < ref_rotor_velocities.size(); i++)
           actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
        actuator_msg->header.stamp = act_header.stamp;

        publish_actuators.publish(actuator_msg);

        // ---------------------------------------------------------------------------------
        // SEZIONE LOOP

        loop_rate.sleep();

        // IMPORT DATI
        ros::spinOnce();
    }


}
