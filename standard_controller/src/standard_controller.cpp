#include "controllore.h"
#include "pid.h"
#include "quat2rpy.h"
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <mav_msgs/default_topics.h>
#include <ros/console.h>
#include <time.h>
#include <chrono>

#include <stdio.h>
#include <math.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/DroneState.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <sensor_msgs/Imu.h>
#include <ros/callback_queue.h>

#define M_PI    3.14159265358979323846  /* pi */

// --------------------------------------------------------------
// COSTANTI
// Sampling Time
double delta_t = 0.01; // [s]

// Parametri veicolo
double maxRotorsVelocity = 2618; // [rad/s]
double Omega_e = 6874; // [rad/s]

// Parametri controllore
double Kp_YawPositionController = 0.0914; //*100
double Kp_ThetaC = 3.594;
double Kp_PhiC = -3.594;
double Kp_pc = 1;
double Kp_qc = 1;
double Kp_altitude = 1700; //1700 e 170
double Kp_deltaPhi = 1000*5; //5000
double Kp_deltaTheta = 1000*5; //5000
double Kp_deltaPsi = 1000;

double Ki_ThetaC = 0;
double Ki_PhiC = -0;
double Ki_pc = 0.0;
double Ki_qc = 0.0;
double Ki_altitude = 3.15;
double Ki_deltaPsi = 95.683;

double Kd_altitude = -373*5; // *5

// Coefficienti modellazione motore
double MotorsIntercepts = 426.24; //[rad/s]
double MotorAngularCoefficient = 0.2685;

// Limiti PID
double yawRateBound = 3.4907; //[rad/s]
double thetaCBound = M_PI/4; //[rad] quarti
double phiCBound = M_PI/4; //[rad]
double deltaOmegaPosBound = 1289; //[PWM]
double deltaOmegaNegBound = -1718; //[PWM]

// DATI IN ACQUISIZIONE
// Riferimento
double x_ref, y_ref, z_ref = 0.0;
double psi_ref = 0.0;
// Posizione attuale
double x_GAZEBO, y_GAZEBO, z_GAZEBO; // Posizione
double u_GAZEBO, v_GAZEBO, w_GAZEBO; // Velocità lineari
double q0_GAZEBO, q1_GAZEBO, q2_GAZEBO, q3_GAZEBO; // x,y,z,w  Quaternione, orientamento del robot
double p_GAZEBO, q_GAZEBO, r_GAZEBO; // Velocità angolari nel frame ABC

// CONTROLLORI PID
PID PID_ThetaC = PID(delta_t, thetaCBound, -thetaCBound, Kp_ThetaC, 0, Ki_ThetaC);
PID PID_PhiC = PID(delta_t, phiCBound, -phiCBound, Kp_PhiC, 0, Ki_PhiC);
PID PID_RC = PID(delta_t, yawRateBound, -yawRateBound, Kp_YawPositionController, 0, 0);
PID PID_PC = PID(delta_t, 1.0e100, -1.0e100, Kp_pc, 0, Ki_pc);
PID PID_QC = PID(delta_t, 1.0e100, -1.0e100, Kp_qc, 0, Ki_qc);
PID PID_DeltaPhi = PID(delta_t, 1.0e100, -1.0e100, Kp_deltaPhi, 0, 0);
PID PID_DeltaTheta = PID(delta_t, 1.0e100, -1.0e100, Kp_deltaTheta, 0, 0);
PID PID_DeltaPsi = PID(delta_t, 1.0e100, -1.0e100, Kp_deltaPsi, 0, Ki_deltaPsi);
PID PID_DeltaOmega = PID(delta_t, 1.0e100, -1.0e100, Kp_altitude, 0 , Ki_altitude);

// HEADER PER MSG
std_msgs::Header act_header;

// BOOL APPOGGIO
bool FirstTargetReceived = false;

// ---------------------------------------------------------------------------------------
// FUNZIONI CALLBACK

void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
    x_GAZEBO = odometry_msg->pose.pose.position.x;
    y_GAZEBO = odometry_msg->pose.pose.position.y;
    z_GAZEBO = odometry_msg->pose.pose.position.z;

    q0_GAZEBO = odometry_msg->pose.pose.orientation.x;
    q1_GAZEBO = odometry_msg->pose.pose.orientation.y;
    q2_GAZEBO = odometry_msg->pose.pose.orientation.z;
    q3_GAZEBO = odometry_msg->pose.pose.orientation.w;

    u_GAZEBO = odometry_msg->twist.twist.linear.x;
    v_GAZEBO = odometry_msg->twist.twist.linear.y;
    w_GAZEBO = odometry_msg->twist.twist.linear.z;

    p_GAZEBO = odometry_msg->twist.twist.angular.x;
    q_GAZEBO = odometry_msg->twist.twist.angular.y;
    r_GAZEBO = odometry_msg->twist.twist.angular.z;

    act_header.stamp = odometry_msg->header.stamp;

}

void ReferenceCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& point_msg)
{
    if (!FirstTargetReceived) {FirstTargetReceived = true;}

    x_ref = point_msg->points[0].transforms[0].translation.x;
    y_ref = point_msg->points[0].transforms[0].translation.y;
    z_ref = point_msg->points[0].transforms[0].translation.z;

    ROS_INFO ("Messaggio di target %f %f %f", x_ref, y_ref, z_ref);
}

// --------------------------------------------------------------------------------------
// FUNZIONE MAIN

int main (int argc, char **argv)
{
    ros::init(argc, argv, "standard_controller");

    ros::NodeHandle n;

    ros::Subscriber sub1 = n.subscribe("odometry", 1, OdometryCallback);
    ros::Subscriber sub2 = n.subscribe("command/trajectory", 1, ReferenceCallback);
    ros::Publisher publish_actuators = n.advertise<mav_msgs::Actuators>("command/motor_speed", 1);

    ros::Rate loop_rate(1/delta_t);

    while (!FirstTargetReceived) {
        ros::spinOnce();
    }

    ROS_INFO("Ricevuto primo target!");

    while (ros::ok())
    {
        // -------------------------------------------------------------------------------
        // SEZIONE ALGORITMO
        // Variabili interne algoritmo
        double yaw, pitch, roll;
        double x_e, y_e;
        double phiC, thetaC;
        double p_c, q_c;
        double delta_phi, delta_theta, delta_psi;
        double r_c;
        double delta_omega;
        double OMEGA;
        double PWM1, PWM2, PWM3, PWM4;
        double W1, W2, W3, W4;

        // CONVERSIONE DA QUATERNIONE A ANGOLI EULERICI RPY
        quat2rpy(q0_GAZEBO, q1_GAZEBO, q2_GAZEBO, q3_GAZEBO, roll, pitch, yaw);

        // Algoritmo di controllo
        ErrorBodyFrame(x_ref, x_GAZEBO, y_ref, y_GAZEBO, yaw, x_e, y_e);

        XYController(x_e, u_GAZEBO, y_e, v_GAZEBO, PID_ThetaC, PID_PhiC, thetaC, phiC);

        AttitudeController(phiC, roll, thetaC, pitch, PID_PC, PID_QC, p_c, q_c);

        YawPositionController(psi_ref, yaw, PID_RC, r_c);

        HoveringController(z_ref, z_GAZEBO, w_GAZEBO, Kd_altitude, deltaOmegaPosBound, deltaOmegaNegBound, PID_DeltaOmega, delta_omega);
        OMEGA = delta_omega + Omega_e;

        RateController(q_c, q_GAZEBO, p_c, p_GAZEBO, r_c, r_GAZEBO, PID_DeltaTheta, PID_DeltaPhi, PID_DeltaPsi,delta_theta, delta_phi, delta_psi);

        ControlMixer(delta_theta, delta_phi, delta_psi, OMEGA, PWM1, PWM2, PWM3, PWM4);

        Motor(PWM1, PWM2, PWM3, PWM4, MotorAngularCoefficient, MotorsIntercepts, maxRotorsVelocity, W1, W2, W3, W4);

        // --------------------------------------------------------------------------------
        // SEZIONE PUBBLICAZIONE
        Eigen::Vector4d ref_rotor_velocities(W1, W2, W3, W4);

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

    return 0;
}
