#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <std_msgs/Float64.h>
#include <math.h>
#include <string>

#define RAD2DEG 57.295779513

float encode_1_vel, encode_2_vel;
float encode_vel, encode_th;
float vel_right, vel_left;
int a,b;

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

int main(int argc, char** argv) {
    // Node initialization
    ros::init(argc, argv, "encoder_orb_odom");
    ros::NodeHandle n;

    tf::TransformBroadcaster odom_broadcaster;
	
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

    // Topic subscribers
    ros::Subscriber vel_1 = n.subscribe("velA", 1000, &vel_A);
    ros::Subscriber vel_2 = n.subscribe("velB", 1000, &vel_B);
    ros::Subscriber encode_1 = n.subscribe("encoder1_value", 1000, &encode1_vel);
    ros::Subscriber encode_2 = n.subscribe("encoder2_value", 1000, &encode2_vel);
    
    float x = 0;
    float y = 0;
    float th = 0;
    float vx = 0;
    float vy = 0;
    float vth = 0;
	
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    int i = 0;
	
    // Node execution
    while(n.ok())
    {
	current_time = ros::Time::now(); 
	double dt = (current_time - last_time).toSec();
	encode_vel = (encode_1_vel + encode_2_vel) / 2;
	encode_th = (encode_1_vel - encode_2_vel) / 0.27;

        vx = encode_vel * cos(th) * dt;
	vy = encode_vel * sin(th) * dt;
	vth = encode_th * dt;

	x = x + vx;
	y = y + vy;
	th = th + vth;
 
	geometry_msgs::TransformStamped odom_trans;
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
		
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";
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
	odom.child_frame_id = "base_link";
	// Linear velocities
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.linear.z = 0.0;
	// Angular velocities
	odom.twist.twist.angular.x = 0;
	odom.twist.twist.angular.y = 0;
	odom.twist.twist.angular.z = vth;
		
	//publish the message
	odom_pub.publish(odom);
		
	last_time = current_time;
	ros::spinOnce();
        i++;
    }
    return 0;
}
