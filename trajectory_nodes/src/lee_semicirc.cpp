#include <thread>
#include <chrono>
#include <math.h>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

double xd, yd, zd;
double x_b1, y_b1, z_b1;
double Ts = 0.01;

#define M_PI    3.14159265358979323846  /* pi */

int main(int argc, char** argv) {

  ros::init(argc, argv, "square_example");

  ros::NodeHandle nh;

  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");

  ros::Publisher trajectory_pub =
      nh.advertise<geometry_msgs::Pose>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  ROS_INFO("Started square example.");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();

  geometry_msgs::Pose trajectory_msg;
  ros::Rate loop_rate(1/Ts);
  double j;

  for (j = 1; j <= 200; j++)
  {
      xd = 0;
      yd = 0;
      zd = j/200 + 0.015;

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

  for (j = 1; j <= 200; j++)
  {
      xd = j/200;
      yd = 0;
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

  ros::Duration(2.0).sleep();

  for (j = 1; j <= 500; j++)
  {
      xd = cos(j*M_PI/500);
      yd = sin(j*M_PI/500);
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
