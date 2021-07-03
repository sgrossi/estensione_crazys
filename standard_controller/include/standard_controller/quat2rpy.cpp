#include "transform_datatypes.h"
#include "Matrix3x3.h"
#include "Quaternion.h"

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>

#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <stdio.h>
#include <boost/bind.hpp>

void quat2rpy(double q0, double q1, double q2, double q3, double& c_roll, double& c_pitch, double& c_yaw) {
    // the incoming message is transformed to a tf::Quaterion

    tf::Quaternion q(q0, q1, q2,
                     q3);
    tf::Matrix3x3 m(q);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    m.getRPY(c_roll, c_pitch, c_yaw);

    }
