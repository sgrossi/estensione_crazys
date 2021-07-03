#include <thread>
#include <chrono>
#include <math.h>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "square_example");

  ros::NodeHandle nh;

  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");

  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  ROS_INFO("Started square example.");
  unsigned int i = 0;

  /* std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);

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
  ros::Duration(5.0).sleep(); */

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();

  // Default desired position and yaw.
  Eigen::Vector3d desired_position(0.0, 0.0, 1.0);
  double desired_yaw = 0.0;

  // Overwrite defaults if set as node parameters.
  nh_private.param("x", desired_position.x(), desired_position.x());
  nh_private.param("y", desired_position.y(), desired_position.y());
  nh_private.param("z", desired_position.z(), desired_position.z());
  nh_private.param("yaw", desired_yaw, desired_yaw);

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);

  ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
           nh.getNamespace().c_str(), desired_position.x(),
           desired_position.y(), desired_position.z());
  trajectory_pub.publish(trajectory_msg);

  ros::spinOnce();

  // ----------------------------------------------------------------------------
  // Dopo 10 secondi sposto il robot
  ros::Duration(10.0).sleep();

  trajectory_msg.header.stamp = ros::Time::now();

  // Default desired position and yaw.
  desired_position[1] = 1.0;

  // Overwrite defaults if set as node parameters.
  nh_private.param("x", desired_position.x(), desired_position.x());
  nh_private.param("y", desired_position.y(), desired_position.y());
  nh_private.param("z", desired_position.z(), desired_position.z());
  nh_private.param("yaw", desired_yaw, desired_yaw);

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);

  ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
           nh.getNamespace().c_str(), desired_position.x(),
           desired_position.y(), desired_position.z());
  trajectory_pub.publish(trajectory_msg);

  ros::spinOnce();

  // ----------------------------------------------------------------------------
  // Dopo 45 secondi faccio una semirotazione
  ros::Duration(40.0).sleep();

  double angle_alpha = 1.5708;
  double angle_arc = (4.71239 - angle_alpha) / 12;

  for (int i = 1; i <= 13; i++)
  {
      desired_position[0] = cos(angle_alpha);
      desired_position[1] = sin(angle_alpha);

      // Overwrite defaults if set as node parameters.
      nh_private.param("x", desired_position.x(), desired_position.x());
      nh_private.param("y", desired_position.y(), desired_position.y());
      nh_private.param("z", desired_position.z(), desired_position.z());
      nh_private.param("yaw", desired_yaw, desired_yaw);

      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
          desired_position, desired_yaw, &trajectory_msg);

      ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
               nh.getNamespace().c_str(), desired_position.x(),
               desired_position.y(), desired_position.z());
      trajectory_pub.publish(trajectory_msg);

      ros::spinOnce();

      angle_alpha = angle_alpha + angle_arc;
      ros::Duration(3.0).sleep();
  }

  ros::spinOnce();
  ros::shutdown();

  return 0;
}
