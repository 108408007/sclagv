#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include "std_msgs/String.h"

class agv_position {
public:
    ros::Publisher odom_pub;
    ros::Subscriber pose_msg;

    void PosCB(const geometry_msgs::Pose2D::ConstPtr& msg);
};


void agv_position::PosCB(const geometry_msgs::Pose2D::ConstPtr& msg) {

  ros::Time current_time;

  current_time = ros::Time::now();

  float x  = msg->x;
  float y  = msg->y;
  float th = msg->theta;

  //ROS_INFO(msg->data.c_str());

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  odom.child_frame_id = "base_link";

  //publish the message
  odom_pub.publish(odom);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "agv_odom");

  ros::NodeHandle n;

  agv_position pose;

  pose.odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  pose.pose_msg = n.subscribe("pose2D", 1000, &agv_position::PosCB, &pose);

  ros::spin();
}
