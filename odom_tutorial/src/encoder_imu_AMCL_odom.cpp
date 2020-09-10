#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <string>

#define RAD2DEG 57.295779513

float encode_1_vel, encode_2_vel;
float encode_vel, encode_th;
float vel_right, vel_left;
float ax, ay, ath;
float x, y, th,vth,imu_th;
int a,b;

double aroll, apitch, ayaw;
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
  encode_1_vel = msg->data * 0.01 * a;
  ROS_INFO("encoder1 = %f", encode_1_vel);
}

void encode2_vel(const std_msgs::Float64::ConstPtr& msg)
{
  encode_2_vel = msg->data * 0.01 *b;
  ROS_INFO("encoder2 = %f", encode_2_vel);
}

void gyr_z(const geometry_msgs::Vector3::ConstPtr& ypr)
{
  vth = ypr->z;
}


void initial_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  // Camera position in map frame
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    // Orientation quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    m.getRPY(roll, pitch, yaw);
    th = yaw;
}

void amcl_location(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  // Camera position in map frame
    ax = msg->pose.pose.position.x;
    ay = msg->pose.pose.position.y;

    // Orientation quaternion
    tf2::Quaternion p(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 n(p);

    // Roll Pitch and Yaw from rotation matrix
    n.getRPY(aroll, apitch, ayaw);
    ath = ayaw;
}

int main(int argc, char** argv) {
    // Node initialization
    ros::init(argc, argv, "encoder_imu_odom");
    ros::NodeHandle n;

    tf::TransformBroadcaster odom_broadcaster;
	
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom1", 50);

    // Topic subscribers
    ros::Subscriber vel_1 = n.subscribe("velA", 1000, &vel_A);
    ros::Subscriber vel_2 = n.subscribe("velB", 1000, &vel_B);

    ros::Subscriber imu_gyr_z = n.subscribe("imu_gyr",1000, &gyr_z);

    ros::Subscriber encode_1 = n.subscribe("encoder1_value", 1000, &encode1_vel);
    ros::Subscriber encode_2 = n.subscribe("encoder2_value", 1000, &encode2_vel);
    ros::Subscriber initialpose = n.subscribe("initialpose", 1000, &initial_pose);
    ros::Subscriber amcl_position = n.subscribe("amcl_pose",10,&amcl_location);
    float vx = 0;
    float vy = 0;
    
	
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    int i = 0;
    ros::Rate loop_rate(50);
    // Node execution
    while(n.ok())
    {   
        if(i<500){
	current_time = ros::Time::now(); 
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
        i++;
        }else{
        x = ax;
        y = ay;
        th = ath;
        i = 0;
        }

 
	geometry_msgs::TransformStamped odom_trans;
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
		
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";
	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
		
	//send the transform over /tf
	odom_broadcaster.sendTransform(odom_trans);  
        
	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";
		
	//set the position
	// Position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	// Orientation
	odom.pose.pose.orientation = odom_quat;
		
	//set the velocity
	odom.child_frame_id = "base_footprint";
	// Linear velocities
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.linear.z = 0.0;
	// Angular velocities
	odom.twist.twist.angular.x = 0;
	odom.twist.twist.angular.y = 0;
	odom.twist.twist.angular.z = 0;
		
	//publish the message
	odom_pub.publish(odom);
		
	last_time = current_time;
	ros::spinOnce();
        loop_rate.sleep();
        i++;
    }
    return 0;
}
