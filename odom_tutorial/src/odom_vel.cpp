#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>
#include <math.h>

float x;
float y;
float th;

float vx = 0;
float vy = 0;
float vth = 0;

float x_1, y_1, th_1;
float x_2, y_2, th_2;

float encode_1_vel;
float encode_2_vel;
float encode_vel;
float encode_th;

float vel_right;
float vel_left;

int i = 0;
int a, b;

ros::Time current_time;
ros::Time last_time;

class agv_position {
public:
    ros::NodeHandle n;

    ros::Publisher odom_pub;
    ros::Subscriber pose_msg;
    ros::Subscriber encode_1;
    ros::Subscriber encode_2;
    ros::Subscriber vel_1;
    ros::Subscriber vel_2;

    tf::TransformBroadcaster odom_broadcaster;

    void PosCB(const geometry_msgs::Pose2D::ConstPtr& msg);
    void encode1_vel(const std_msgs::Float64::ConstPtr& msg);
    void encode2_vel(const std_msgs::Float64::ConstPtr& msg);
    void vel_A(const std_msgs::Float64::ConstPtr& msg);
    void vel_B(const std_msgs::Float64::ConstPtr& msg);
};

void agv_position::vel_A(const std_msgs::Float64::ConstPtr& msg)
{ 
  vel_right = msg->data;
  if(vel_right < 0)
  {
    a = -1;
  }
  else
  {
    a = 1;
  }
}

void agv_position::vel_B(const std_msgs::Float64::ConstPtr& msg)
{ 
  vel_left = msg->data;
  if(vel_left < 0)
  {
    b = -1;
  }
  else
  {
    b = 1;
  }
}

void agv_position::encode2_vel(const std_msgs::Float64::ConstPtr& msg)
{
  encode_2_vel = msg->data * 0.01 * b;
}


void agv_position::encode1_vel(const std_msgs::Float64::ConstPtr& msg)
{
  encode_1_vel = msg->data * 0.01 * a;
}


void agv_position::PosCB(const geometry_msgs::Pose2D::ConstPtr& msg) 
{
  agv_position pose;
  
  current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();

  tf::TransformBroadcaster odom_broadcaster;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_footprint";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  pose.odom_broadcaster.sendTransform(odom_trans);

// amcl broke, grab two data to compare, if amcl broke(means my pose is at wrong place when second data input,then first data will replace second data)
  if(i % 2 == 1)
  {
     x_1  = msg->x;
     y_1  = msg->y;
     th_1 = msg->theta;

     vx = x_1 - x_2;
     vy = y_1 - y_2;
     vth = th_1 - th_2;
  }
  else
  {
     x_2  = msg->x;
     y_2  = msg->y;
     th_2 = msg->theta;

     vx = x_2 - x_1;
     vy = y_2 - y_1;
     vth = th_2 - th_1;
  }

  bool vel_error = vx > 0.3 | vy > 0.3 | vx < -0.3 | vy < -0.3 | vth > 1 | vth < -1;
 
  if(vel_error == 1)
  {
     ROS_INFO("change encoder");
     encode_vel = (encode_1_vel + encode_2_vel) / 2;
     encode_th = (encode_1_vel - encode_2_vel) / 0.27;
     vx = encode_vel * cos(th) * dt;
     vy = encode_vel * sin(th) * dt;
     vth = encode_th * dt;
  }

  x += vx;
  y += vy;
  th += vth;

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_footprint";

  //publish the message
  odom_pub.publish(odom);
  i++;
  last_time = current_time;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odom_vel");

  agv_position pose;

  current_time = ros::Time::now();

  pose.odom_pub = pose.n.advertise<nav_msgs::Odometry>("odom", 50);

  pose.vel_1 = pose.n.subscribe("velA", 1000, &agv_position::vel_A, &pose);
  pose.vel_2 = pose.n.subscribe("velB", 1000, &agv_position::vel_B, &pose);
  pose.encode_1 = pose.n.subscribe("encoder1_value", 1000, &agv_position::encode1_vel, &pose);
  pose.encode_2 = pose.n.subscribe("encoder2_value", 1000, &agv_position::encode2_vel, &pose);
  pose.pose_msg = pose.n.subscribe("pose2D", 1000, &agv_position::PosCB, &pose);
  ros::spin();
}
