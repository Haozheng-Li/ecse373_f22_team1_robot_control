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
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Subscriber sub = n.subscribe("des_vel", 1000, desiredVelCallback);
  ros::Subscriber sub1 = n.subscribe("robot0/laser_1", 1000, laserScanCallback);

  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1000);
  p_cmd_vel_pub = &cmd_vel_pub;

  ros::Rate loop_rate(10);

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