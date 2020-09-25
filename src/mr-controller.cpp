#include "ros/ros.h"
#include <tf/tf.h>
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include "hybridAutomata.h"


double x_d=2.0, y_d=-1.0;

//initializing and declaring variables for robot's current pose
double x=0,y=0,roll=0,pitch=0,yaw=0;
double range[360];

void readLaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  for(int i=0;i<360;i++)
  {
    if(msg->ranges[i] > msg->range_max)
      range[i] = 4.0;
    else
      range[i]=msg->ranges[i];
  }
}


void robotPoseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  x=msg->pose.pose.position.x;
  y=msg->pose.pose.position.y;
  tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mr_controller");
    ros::NodeHandle n;

    ros::Subscriber odometry = n.subscribe("odom", 1000, robotPoseCallback);
    ros::Subscriber laserScan = n.subscribe("scan", 10, readLaserCallback);

    // Command Velocity Publisher
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    
    //Running at 10Hz
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        hybrid_automata HA;
        geometry_msgs::Twist velo = HA.switcher(x,y,yaw,x_d,y_d,range);
        vel_pub.publish(velo);
    
        //For running the callbacks
        ros::spinOnce();
        //wait till 10Hz loop rate is satified.
        loop_rate.sleep();
    }

    return 0;
}