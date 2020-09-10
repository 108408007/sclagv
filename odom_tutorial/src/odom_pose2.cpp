#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <string>

#define RAD2DEG 57.295779513

/**
 * This tutorial demonstrates receiving ZED odom and pose messages over the ROS system.
 */
double tx, ty, tz;
double roll, pitch, yaw;


/**
 * Subscriber callbacks
 */

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    // Camera position in map frame
    tx = msg->pose.position.x;
     ty = msg->pose.position.y;
     tz = msg->pose.position.z;

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

    // Output the measure
    ROS_INFO("Received pose in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
             msg->header.frame_id.c_str(),
             tx, ty, tz,
             roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
}

/**
 * Node main function
 */
int main(int argc, char** argv) {
    // Node initialization
    ros::init(argc, argv, "odom_pose");
    ros::NodeHandle n;
    tf::TransformBroadcaster odom_broadcaster;

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom0", 50);

    // Topic subscribers
    ros::Subscriber subPose = n.subscribe("/orb_slam2_rgbd/pose", 10, &poseCallback);

    ros::Rate loop_rate(10);

    while(n.ok())
    {
       ros::spinOnce();
       geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

       //first, we'll publish the transform over tf
       geometry_msgs::TransformStamped odom_trans;
       odom_trans.header.stamp = ros::Time::now();
       odom_trans.header.frame_id = "orb_odom";
       odom_trans.child_frame_id = "base_footprint";

       odom_trans.transform.translation.x = tx - 0.275;
       odom_trans.transform.translation.y = ty;
       odom_trans.transform.translation.z = -0.66;
       odom_trans.transform.rotation = odom_quat;

       //send the transform
       odom_broadcaster.sendTransform(odom_trans);
 
       nav_msgs::Odometry odom;
       odom.header.stamp = ros::Time::now();
       odom.header.frame_id = "orb_odom";

       //set the position
       odom.pose.pose.position.x = tx - 0.275;
       odom.pose.pose.position.y = ty;
       odom.pose.pose.position.z = -0.66;
       odom.pose.pose.orientation = odom_quat;
       odom.child_frame_id = "base_footprint";

       //publish the message
       odom_pub.publish(odom);
       loop_rate.sleep();
    }
    return 0;
}
