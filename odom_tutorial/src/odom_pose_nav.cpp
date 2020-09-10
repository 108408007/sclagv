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

class agv_position {
public:
    ros::Publisher odom_pub;
    ros::Subscriber subPose;

    tf::TransformBroadcaster odom_broadcaster;

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

/**
 * Subscriber callbacks
 */

void agv_position::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    agv_position pose;

    // Camera position in map frame
    double tx = msg->pose.position.x - 0.26;
    double ty = msg->pose.position.y - 0.05;
    double tz = msg->pose.position.z;

    // Orientation quaternion
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Output the measure
    ROS_INFO("Received pose in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
             msg->header.frame_id.c_str(),
             tx, ty, tz,
             roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);

    ros::Time current_time;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = tx;
    odom_trans.transform.translation.y = ty;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    pose.odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "map";

    //set the position
    odom.pose.pose.position.x = tx;
    odom.pose.pose.position.y = ty;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.child_frame_id = "base_link";

    //publish the message
    odom_pub.publish(odom);
}

/**
 * Node main function
 */
int main(int argc, char** argv) {
    // Node initialization
    ros::init(argc, argv, "odom_pose_nav");
    ros::NodeHandle n;
    agv_position pose;

    pose.odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

    // Topic subscribers
    pose.subPose = n.subscribe("/orb_slam2_rgbd/pose", 10, &agv_position::poseCallback, &pose);

    // Node execution
    ros::spin();

    return 0;
}
