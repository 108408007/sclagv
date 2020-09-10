#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <math.h>
#include <string>

#define RAD2DEG 57.295779513

/**
 * This tutorial demonstrates receiving ZED odom and pose messages over the ROS system.
 */
double orb_x, orb_y, orb_z;
double orb_roll, orb_pitch, orb_yaw;

float encode_1_vel, encode_2_vel;
float encode_vel, encode_th;
float vel_right, vel_left;

float vx, vy, vth, imu_th;
int a,b;

float x, y, th;

int localization;


/**
 * Subscriber callbacks
 */
void change_localization(const std_msgs::Int32::ConstPtr& msg)
{
    localization = msg->data;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
    // Camera position in map frame
    orb_x = msg->pose.position.x;
    orb_y = msg->pose.position.y;
    orb_z = msg->pose.position.z;

    // Orientation quaternion
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    m.getRPY(orb_roll, orb_pitch, orb_yaw);

    // Output the measure
    ROS_INFO("Received pose in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
             msg->header.frame_id.c_str(),
             orb_x, orb_y, orb_z,
             orb_roll * RAD2DEG, orb_pitch * RAD2DEG, orb_yaw * RAD2DEG);
}

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
  encode_1_vel = msg->data * 0.011 * a;
  //ROS_INFO("encoder1 = %f", encode_1_vel);
}

void encode2_vel(const std_msgs::Float64::ConstPtr& msg)
{
  encode_2_vel = msg->data * 0.011 *b;
  //ROS_INFO("encoder2 = %f", encode_2_vel);
}

void gyr_z(const geometry_msgs::Vector3::ConstPtr& ypr)
{
  vth = ypr->z;
}

/**
 * Node main function
 */
int main(int argc, char** argv) {
    // Node initialization
    ros::init(argc, argv, "odom_navigation");
    ros::NodeHandle n;
    tf::TransformBroadcaster odom_broadcaster;

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom0", 50);

    // Topic subscribers
    ros::Subscriber subPose = n.subscribe("/orb_slam2_rgbd/pose", 10, &poseCallback);
    ros::Subscriber vel_1 = n.subscribe("velA", 1000, &vel_A);
    ros::Subscriber vel_2 = n.subscribe("velB", 1000, &vel_B);

    ros::Subscriber imu_gyr_z = n.subscribe("imu_gyr",1000, &gyr_z);

    ros::Subscriber encode_1 = n.subscribe("encoder1_value", 1000, &encode1_vel);
    ros::Subscriber encode_2 = n.subscribe("encoder2_value", 1000, &encode2_vel);
    ros::Subscriber change_local = n.subscribe("/change_localization", 1000, &change_localization);

    vx = 0;
    vy = 0;
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate loop_rate(10);
    while(n.ok())
    {
       ros::spinOnce();
       current_time = ros::Time::now(); 

       if(localization == 0)
       {
          ros::Rate loop_rate(10);
          x = orb_x;
          y = orb_y;
          th = orb_yaw;
       }
       else
       {
          ros::Rate loop_rate(50);
          double dt = (current_time - last_time).toSec();
	  encode_vel = (encode_1_vel + encode_2_vel) / 2;
	  imu_th = vth *dt;

          vx = encode_vel * cos(th) * dt;
	  vy = encode_vel * sin(th) * dt;

          x = x + vx;
	  y = y + vy;
          th = th + imu_th;

          int th_num;
          th_num = th / 6.2832;
          th = th - (th_num * 6.2832);
       }

       geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

       //first, we'll publish the transform over tf
       geometry_msgs::TransformStamped odom_trans;
       odom_trans.header.stamp = current_time;

       odom_trans.header.frame_id = "odom";
       odom_trans.child_frame_id = "base_footprint";
       odom_trans.transform.translation.x = x;
       odom_trans.transform.translation.y = y;
       odom_trans.transform.translation.z = -0.66;
       odom_trans.transform.rotation = odom_quat;
 
       nav_msgs::Odometry odom;
       odom.header.stamp = current_time;
       odom.header.frame_id = "odom";
       odom.child_frame_id = "base_footprint";
       odom.pose.pose.position.x = x;
       odom.pose.pose.position.y = y;
       odom.pose.pose.position.z = -0.66;
       odom.pose.pose.orientation = odom_quat;

       //send the transform
       odom_broadcaster.sendTransform(odom_trans);
       //publish the message
       odom_pub.publish(odom);

       last_time = current_time;
       loop_rate.sleep();
    }
    return 0;
}
