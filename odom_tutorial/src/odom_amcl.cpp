#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

float encode_1_vel, encode_2_vel;
float encode_vel, encode_th;
float vel_right, vel_left;

float x, y, th,vth,imu_th;
int a,b;

float encoder_x;
float encoder_y;
float encoder_th;

float amcl_x;
float amcl_y;
float amcl_th;

double roll, pitch, yaw;

void vel_A(const std_msgs::Float64::ConstPtr& msg)
{ 
  vel_right = msg->data;
  if(vel_right < 0)
    a = -1;
  else
    a = 1;
}

void vel_B(const std_msgs::Float64::ConstPtr& msg)
{ 
  vel_left = msg->data;
  if(vel_left < 0)
    b = -1;
  else
    b = 1;
}

void encode1_vel(const std_msgs::Float64::ConstPtr& msg)
{
  encode_1_vel = msg->data * 0.0105 * a;
  ROS_INFO("encoder1 = %f", encode_1_vel);
}

void encode2_vel(const std_msgs::Float64::ConstPtr& msg)
{
  encode_2_vel = msg->data * 0.0105 *b;
  ROS_INFO("encoder2 = %f", encode_2_vel);
}

void gyr_z(const geometry_msgs::Vector3::ConstPtr& ypr)
{
  vth = ypr->z;
}

void amcl_poscb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
  amcl_x = msgAMCL->pose.pose.position.x;
  amcl_y = msgAMCL->pose.pose.position.y;

   tf2::Quaternion q(
        msgAMCL->pose.pose.orientation.x,
        msgAMCL->pose.pose.orientation.y,
        msgAMCL->pose.pose.orientation.z,
        msgAMCL->pose.pose.orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    m.getRPY(roll, pitch, yaw);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odom_amcl");
  ros::NodeHandle n;

  tf::TransformBroadcaster odom_broadcaster;

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  // Topic subscribers
  ros::Subscriber vel_1 = n.subscribe("velA", 1000, &vel_A);
  ros::Subscriber vel_2 = n.subscribe("velB", 1000, &vel_B);

  ros::Subscriber imu_gyr_z = n.subscribe("imu_gyr",1000, &gyr_z);

  ros::Subscriber encode_1 = n.subscribe("encoder1_value", 1000, &encode1_vel);
  ros::Subscriber encode_2 = n.subscribe("encoder2_value", 1000, &encode2_vel);

  ros::Subscriber amcl_pose_msg = n.subscribe("amcl_pose", 1000, &amcl_poscb);

  ros::Rate loop_rate(10);

  ros::Time inition_time = ros::Time::now();
  ros::Time current_time, last_time;

  while(n.ok())
  {
     ros::spinOnce();
     current_time = ros::Time::now();

     double continue_time = (current_time - inition_time).toSec();

     geometry_msgs::TransformStamped odom_trans;
     geometry_msgs::Quaternion odom_quat;

     odom_trans.header.stamp = current_time;
     odom_trans.header.frame_id = "odom";
     odom_trans.child_frame_id = "base_footprint";

     nav_msgs::Odometry odom;
     odom.header.stamp = current_time;
     odom.header.frame_id = "odom";
     odom.child_frame_id = "base_footprint";
     
     if(continue_time < 7)
     {
        odom_quat = tf::createQuaternionMsgFromYaw(encoder_th);

        double dt = (current_time - last_time).toSec();
        encode_vel = (encode_1_vel + encode_2_vel) / 2;
	imu_th = vth *dt;

        double vx = encode_vel * cos(th) * dt;
	double vy = encode_vel * sin(th) * dt;
	

	encoder_x = encoder_x + vx;
	encoder_y = encoder_y + vy;
        encoder_th = encoder_th + imu_th;

        int th_num;
        th_num = encoder_th / 6.2832;
        encoder_th = encoder_th - (th_num * 6.2832);

        //first, we'll publish the transform over tf
        odom_trans.transform.translation.x = encoder_x;
        odom_trans.transform.translation.y = encoder_y;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = odom_quat;

        //set the position
        odom.pose.pose.position.x = encoder_x;
        odom.pose.pose.position.y = encoder_y;
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = odom_quat;
     }
     else
     {
        odom_quat = tf::createQuaternionMsgFromYaw(yaw);
        //first, we'll publish the transform over tf
        odom_trans.transform.translation.x = amcl_x;
        odom_trans.transform.translation.y = amcl_y;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = odom_quat;

        //set the position
        odom.pose.pose.position.x = amcl_x;
        odom.pose.pose.position.y = amcl_y;
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = odom_quat;
     }

     //send the transform
     odom_broadcaster.sendTransform(odom_trans);

     //publish the message
     odom_pub.publish(odom);
     last_time = current_time;
     loop_rate.sleep();
  }
  return 0;
}
