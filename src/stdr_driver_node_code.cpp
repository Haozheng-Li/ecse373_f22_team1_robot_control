#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>

// Pointer Definition
ros::Publisher *p_cmd_vel_pub;


// Callback Function Definition
void desiredVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ROS_INFO("Forward velocity %2.2f; Angular velocity %2.2f", msg->linear.x, msg->linear.z);
  p_cmd_vel_pub->publish(*msg);
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("LaserScan sequence number is: %i", msg->header.seq);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  // Init the node
  ros::NodeHandle n;

  // Init the subscriber
  ros::Subscriber sub = n.subscribe("des_vel", 1000, desiredVelCallback);
  ros::Subscriber sub1 = n.subscribe("robot0/laser_1", 1000, laserScanCallback);
  
  // Init the publisher
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1000);


  p_cmd_vel_pub = &cmd_vel_pub;

  // See the frequency of loop in the node
  ros::Rate loop_rate(10);

  // Main logic
  while (ros::ok())
  {
    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0.1;

    // cmd_vel_pub.publish(cmd_vel);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}