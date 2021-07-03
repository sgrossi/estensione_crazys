#include <thread>
#include <chrono>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

double xd, yd, zd;
double x_b1, y_b1, z_b1;
double Ts = 0.01;

using namespace Eigen;

int main(int argc, char** argv) {

  ros::init(argc, argv, "lin1_1");

  ros::NodeHandle nh;

  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");

  ros::Publisher trajectory_pub =
      nh.advertise<geometry_msgs::Pose>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(10.0).sleep();

  geometry_msgs::Pose trajectory_msg;
  ros::Rate loop_rate(1/Ts);

  double t = 3;
  Vector4d Q(0,1,0,0);
  Matrix4d G;
  G <<      1,          0,          0,          0,
            1,          t,          t*t,        t*t*t,
            0,          1,          0,          0,
            0,          1,          2*t,        3*t*t;
  Vector4d A;
  A = G.inverse() * Q;

  double p;
  double s;
  double j;

  for (j = 1; j <= t*100; j++)
  {
      p = j / 100;
      s = A(0) + A(1)*p + A(2)*p*p + A(3)*p*p*p;

      xd = 1;
      yd = -1;
      zd = s + 0.015;

      x_b1 = 1;
      y_b1 = 0;
      z_b1 = 0;

      /* ROS_INFO("Iteraz j: %f", j);
      ROS_INFO("Pubblico z: %f", zd); */

      trajectory_msg.position.x = xd;
      trajectory_msg.position.y = yd;
      trajectory_msg.position.z = zd;

      trajectory_msg.orientation.x = x_b1;
      trajectory_msg.orientation.y = y_b1;
      trajectory_msg.orientation.z = z_b1;

      trajectory_pub.publish(trajectory_msg);

      ros::Duration(Ts).sleep();
      ros::spinOnce();
  }

  ros::Duration(3.0).sleep();

  t = 5;
  G <<      1,          0,          0,          0,
            1,          t,          t*t,        t*t*t,
            0,          1,          0,          0,
            0,          1,          2*t,        3*t*t;
  A = G.inverse() * Q;

  for (j = 1; j <= t*100; j++)
  {
      p = j / 100;
      s = A(0) + A(1)*p + A(2)*p*p + A(3)*p*p*p;

      xd = 1;
      yd = s-1;
      zd = 1.015;

      x_b1 = 1;
      y_b1 = 0;
      z_b1 = 0;

      trajectory_msg.position.x = xd;
      trajectory_msg.position.y = yd;
      trajectory_msg.position.z = zd;

      trajectory_msg.orientation.x = x_b1;
      trajectory_msg.orientation.y = y_b1;
      trajectory_msg.orientation.z = z_b1;

      trajectory_pub.publish(trajectory_msg);

      ros::Duration(Ts).sleep();
      ros::spinOnce();
  }

  ros::shutdown();

  return 0;
}
