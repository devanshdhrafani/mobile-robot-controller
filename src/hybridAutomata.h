#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <cmath>

double kP=0.3;
double kP_vel=0.7;
double v0=1.0;
double alpha=0.7;

class hybrid_automata{
    private:
        double phi_desired;
        double goalDistance;

        double euclideanDistance(double x0, double y0, double x1, double y1){
            return sqrt(pow(x0-x1,2)+pow(y0-y1,2));
        }

        geometry_msgs::Twist goToGoal(double x_curr, double y_curr, double yaw_curr, double x_desired, double y_desired)
        {
            geometry_msgs::Twist command_velo;
            double headingError = phi_desired-yaw_curr;
            headingError = atan2(sin(headingError),cos(headingError));
            //headingError = abs(headingError);
            ROS_INFO("Executing GTG, HeadingError: %f, DistToGoal: %f",headingError,goalDistance);

            command_velo.angular.x = 0;
            command_velo.angular.y = 0;
            command_velo.angular.z = kP*headingError; //dividing by pi to normalise error

            command_velo.linear.x = kP_vel*v0*(1-exp(-1*alpha*pow(goalDistance,2))); //for large error, v goes to v0
            command_velo.linear.y = 0;
            command_velo.linear.z = 0;
            
            return command_velo;
        }
        
    public:
        geometry_msgs::Twist switcher(double x_curr, double y_curr, double yaw_curr, double x_desired, double y_desired)
        {
            phi_desired = atan2(y_desired-y_curr, x_desired-x_curr);
            goalDistance = this->euclideanDistance(x_curr,y_curr,x_desired,y_desired);
            if(goalDistance>0.05){
                return this->goToGoal(x_curr,y_curr,yaw_curr,x_desired,y_desired);
            }
            else{
                geometry_msgs::Twist command_velo;
                command_velo.linear.x = 0;
                command_velo.linear.y = 0;
                command_velo.linear.z = 0;
                command_velo.angular.x = 0;
                command_velo.angular.y = 0;
                command_velo.angular.z = 0;
                ROS_INFO("Stopping Bot");
                return command_velo;
            }        
        }
};