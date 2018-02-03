
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "init_pos_publisher.h"
#include "math.h"
#include <tf/transform_broadcaster.h>
#include <string>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "init_pos_publisher_node");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");
  geometry_msgs::PoseWithCovarianceStamped      robot_pos;

  double x_pose;
  double y_pose;
  double yaw;

  priv_nh.param("x_pose", x_pose, 0.0);
  priv_nh.param("y_pose", y_pose, 0.0);
  priv_nh.param("yaw", yaw,    0.0);

  try
  {
    ros::Publisher pos_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1);

    robot_pos.header.frame_id = "/map";
    robot_pos.header.seq = 0;
    robot_pos.header.stamp = ros::Time::now();

    //robot_pos.pose.pose.position.x = 11.7;
    //robot_pos.pose.pose.position.y = 2.9;
    robot_pos.pose.pose.position.x = x_pose;
    robot_pos.pose.pose.position.y = y_pose;

    robot_pos.pose.pose.position.z = 0.0;

    robot_pos.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    ros::Rate loop_rate(10);
    ROS_INFO("Initialize pos publisher started");

    ros::Duration(2.0).sleep(); // sleep for half a second

    pos_publisher.publish(robot_pos);
    ROS_INFO("Init pos published");

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
  }
  catch (int)
  {
    ROS_ERROR("Error\n");
    return -1;
  }
}
