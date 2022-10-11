#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>

#define PI 3.1415926

// Global Variabies Definition
sensor_msgs::LaserScan laser_data;
geometry_msgs::Twist cmd_vel;
bool manual_control = false;

// Callback Function Definition
void desiredVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  manual_control = true;
  cmd_vel = *msg;
  ROS_INFO("Forward velocity %2.2f; Angular velocity %2.2f", msg->linear.x, msg->angular.z);
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  /*
  Format of sensor_msgs/LaserScan
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  float32 angle_min
  float32 angle_max
  float32 angle_increment
  float32 time_increment
  float32 scan_time
  float32 range_min
  float32 range_max
  float32[] ranges
  float32[] intensities
  */
  laser_data = *msg;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "stdr_driver_node");

  // Init the node
  ros::NodeHandle n;

  // Init the subscriber
  ros::Subscriber sub = n.subscribe("des_vel", 1000, desiredVelCallback);
  ros::Subscriber sub1 = n.subscribe("laser_1", 1000, laserScanCallback);
  
  // Init the publisher
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  // Set the frequency of loop in the node
  ros::Rate loop_rate(10);

  float min_distance = 0.5;
  float detect_angle = 70;
  bool almost_hit = false;

  // Main logic
  while (ros::ok())
  {
    almost_hit = false;

    // Velocity initial
    cmd_vel.linear.x = 0.1;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;

    int range_length = ceil((laser_data.angle_max-laser_data.angle_min)/laser_data.angle_increment);

    /*  Hit Judgement 
        Default scan angle from -135 to 135(-2.35 to 2.35)
        we only care about the -70 to 70(face front)
     */
    for (int i=0; i<range_length; i++)
    {
      float scan_angle = laser_data.angle_min + i*laser_data.angle_increment;
      if(scan_angle >= -detect_angle * PI / 180 && scan_angle <= detect_angle * PI / 180)
      {
         if (laser_data.ranges[i] < min_distance)
         {
            almost_hit = true;
         }
      }
    }

    if (almost_hit)
    {
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0.2;
      ROS_INFO("ALMOST HIT, START TURNING");

      manual_control=false; // When the robot get too close to the wall, switch to auto control
    }
    else{
      if (!manual_control){
        cmd_vel.linear.x = 0.1;
        cmd_vel.angular.z = 0;
      }

      ROS_INFO("HAVE ENOUGH DISTANCE, CONTINUE TO MOVE");
    }

    cmd_vel_pub.publish(cmd_vel);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}