#include "ros/ros.h"
#include <tf/tf.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Pose2D.h>
#include <cmath>

//for storing robot pose
double x,y,z,roll,pitch,yaw;
double x_d,y_d,yaw_d,roll_d,pitch_d;

//cmd_vel publisher
ros::Publisher vel_pub;

//PID
double dist=10.0;
double old_e=0;
double E=0;
double kP=0.5;
double kD=0;
double kI=0;
double e=10.0;
double e_dot, omega, x_dot, y_dot;

//is the goal received?
int goals=0;
bool goalReached = false;

//set max velocity
const double v_max=0.1;

void calculateError(){
	//e = atan((y_d-y)/(x_d-x)) - yaw;
	e = atan(yaw_d - yaw);
	dist = sqrt((x-x_d)*(x-x_d)+(y-y_d)*(y-y_d));
	e_dot = e-old_e;
	E = E + e;
	ROS_INFO("Dist: %f, yaw_error: %f",dist,e);
	if(dist<=0.01 && abs(e)<=0.01){
		goalReached = true;
	}
}

void goToGoal(){
	calculateError();
	omega = kP*e + kD*e_dot + kI*E;  //PID
	x_dot=v_max*dist;
	ROS_INFO("Executing GTG with x_dot=%f, y_dot=%f, omega=%f",x_dot,y_dot,omega);
	geometry_msgs::Twist velo;
	velo.linear.x=x_dot;
	velo.angular.z=omega;
	vel_pub.publish(velo);
}

void stopBot(){
	ROS_INFO("Stopping Bot");
	geometry_msgs::Twist velo;
	velo.linear.x=0;
	velo.angular.z=0;
	vel_pub.publish(velo);
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
	if(goals>0 && !goalReached){
		goToGoal();
	}
	else{
		stopBot();
	}
}

void desiredPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  x_d=msg->pose.position.x;
  y_d=msg->pose.position.y;
  tf::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll_d, pitch_d, yaw_d);
	goals++;
  ROS_INFO("New Goal received! (x_d:%f, y_d:%f, yaw_d:%f)",x_d,y_d,yaw_d);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic_controller");
	ros::NodeHandle n;

	//cmd_vel publisher
	vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);

  ros::Subscriber odometry = n.subscribe("odom", 1000, robotPoseCallback);
  ros::Subscriber goal = n.subscribe("move_base_simple/goal", 1000, desiredPoseCallback);

  ros::spin();

  return 0;
}