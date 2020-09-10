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
float ox, oy, oth;
float x, y, th,vth,imu_th;
int a,b;
float sa = 0;
float vx,vy;
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
  encode_1_vel = msg->data * 0.011 * a;
  ROS_INFO("encoder1 = %f", encode_1_vel);
}

void encode2_vel(const std_msgs::Float64::ConstPtr& msg)
{
  encode_2_vel = msg->data * 0.011 *b;
  ROS_INFO("encoder2 = %f", encode_2_vel);
}

void gyr_z(const geometry_msgs::Vector3::ConstPtr& ypr)
{
  vth = ypr->z;
}


void orbpose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // Camera position in map frame
    ox = msg->pose.position.x;
    oy = msg->pose.position.y;

    // Orientation quaternion
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    m.getRPY(roll, pitch, yaw);
    oth = yaw;
}

void readsignal(const std_msgs::Float64::ConstPtr& msg)
{
  //sa = msg->data;
  
}

int main(int argc, char** argv) {
    // Node initialization
    ros::init(argc, argv, "encoder_imu_orb_odom");
    ros::NodeHandle n;

    tf::TransformBroadcaster odom_broadcaster;
	
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom1", 50);

    // Topic subscribers
    ros::Subscriber vel_1 = n.subscribe("velA", 1000, &vel_A);
    ros::Subscriber vel_2 = n.subscribe("velB", 1000, &vel_B);

    ros::Subscriber imu_gyr_z = n.subscribe("imu_gyr",1000, &gyr_z);

    ros::Subscriber encode_1 = n.subscribe("encoder1_value", 1000, &encode1_vel);
    ros::Subscriber encode_2 = n.subscribe("encoder2_value", 1000, &encode2_vel);
    ros::Subscriber orb_pose = n.subscribe("orb_slam2_rgbd/pose", 10, &orbpose);
    ros::Subscriber signalA  = n.subscribe("signal_a",10,&readsignal);
    
    ros::Time current_time, last_time;
    ros::Rate loop_rate(50);
    // Node execution
    while(n.ok())
    { 
      ros::spinOnce();
       
      if(oy<-9){
        sa = 1;
      }

      if(sa ==0)
      {    
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        odom_trans.transform.translation.x = ox - 0.275;
        odom_trans.transform.translation.y = oy;
        odom_trans.transform.translation.z = -0.66;
        odom_trans.transform.rotation = odom_quat;
 
        //send the transform
        odom_broadcaster.sendTransform(odom_trans);
 
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
 
        //set the position
        odom.pose.pose.position.x = ox - 0.275;
        odom.pose.pose.position.y = oy;
        odom.pose.pose.position.z = -0.66;
        odom.pose.pose.orientation = odom_quat;
        odom.child_frame_id = "base_footprint";

        //publish the message
        odom_pub.publish(odom);
        
        x = ox;
        y = oy;
        th = yaw;
        last_time = ros::Time::now();
      } 
      else if (sa ==1)
      { 
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
	
    }
    loop_rate.sleep();
  }
  return 0;
}
