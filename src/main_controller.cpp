#include "ros/ros.h"
#include <tf/tf.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Pose2D.h>
#include <cmath>

ros::Publisher vel_pub;
ros::Subscriber odometry;
ros::Subscriber goal;

//for storing robot pose
double x,y,z,roll,pitch,yaw;
double x_d,y_d,yaw_d,roll_d,pitch_d;

int goals=0;

void rotate (double angular_speed, double angle, bool clockwise);
double degrees2radians(double angle_in_degrees);
void setDesiredOrientation (double desired_angle_radians);
void moveGoal(double distance_tolerance);

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
	if(goals>0){
		moveGoal(0.05);
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
  	ros::init(argc, argv, "main_controller");
	ros::NodeHandle n;

	//cmd_vel publisher
	vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	odometry = n.subscribe("odom", 100, robotPoseCallback);
	goal = n.subscribe("move_base_simple/goal", 100, desiredPoseCallback);

	//ros::Rate loop(0.5);
	//if(goals>0){
	//	moveGoal(0.01);
	//}
	//loop.sleep();

	ros::spin();

	return 0;
}

double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

void moveGoal(double distance_tolerance){
	geometry_msgs::Twist vel_msg;

	ros::Rate loop_rate(100);
	double E = 0.0;
	do{
		/****** Proportional Controller ******/
		//linear velocity in the x-axis
		double Kp=0.5;
		double Ki=0.02;
		//double v0 = 2.0;
		//double alpha = 0.5;
		double e = getDistance(x, y, x_d, y_d);
		double E = E+e;
		//Kp = v0 * (exp(-alpha)*error*error)/(error*error);
		vel_msg.linear.x = (Kp*e);
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;
		//angular velocity in the z-axis
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =2*(atan2(y_d-y, x_d-x)-yaw);

		vel_pub.publish(vel_msg);

		ros::spinOnce();
		loop_rate.sleep();

	}while(getDistance(x, y, x_d, y_d)>distance_tolerance);
	std::cout<<"end move goal\n";
	vel_msg.linear.x =0;
	vel_msg.angular.z = 0;
	vel_pub.publish(vel_msg);
}