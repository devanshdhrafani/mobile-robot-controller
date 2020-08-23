#include "ros/ros.h"
#include <tf/tf.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Pose2D.h>
#include <cmath>

//for storing robot pose
double x,y,z,roll,pitch,yaw;

//PID
double old_e=0;
double E=0;
double kP=0.5;
double kD=0;
double kI=0;

//set max velocity
const double v_max=0.05;

void goToGoal(double x_d, double y_d, double yaw_d){
	//find error
	double e = atan((y_d-y)/(x_d-x));
	double e_dot = e-old_e;
	E = E + e;

	double omega = kP*e + kD*e_dot + kI*E;  //PID
	double x_dot=v_max*cos(yaw);
	double y_dot=v_max*sin(yaw);
	ROS_INFO("Executing GTG with x_dot=%f, y_dot=%f, omega=%f",x_dot,y_dot,omega);
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
  //ROS_INFO("x: %f, y: %f, yaw: %f, %f %f",x,y,yaw,roll,pitch);
}

void desiredPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  double x_d=msg->pose.position.x;
  double y_d=msg->pose.position.y;
  tf::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll_d, pitch_d, yaw_d;
  m.getRPY(roll_d, pitch_d, yaw_d);
  ROS_INFO("New Goal received! (x_d:%f, y_d:%f, yaw_d:%f)",x_d,y_d,yaw_d);
  goToGoal(x_d,y_d,yaw_d);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic_controller");
  ros::NodeHandle n;

  ros::Subscriber odometry = n.subscribe("odom", 1000, robotPoseCallback);
  ros::Subscriber goal = n.subscribe("move_base_simple/goal", 100, desiredPoseCallback);
 
  ros::spin();

  return 0;
}