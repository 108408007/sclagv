#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "vel_pub");

  ros::NodeHandle n;

  ros::Publisher vel_1 = n.advertise<std_msgs::Float64>("encoder1_value", 1000);
  ros::Publisher vel_2 = n.advertise<std_msgs::Float64>("encoder2_value", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std_msgs::Float64 vx;
    std_msgs::Float64 vy;

    vx.data = 0.3;
    vy.data = 0.27;

    vel_1.publish(vx);

    vel_2.publish(vy);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
