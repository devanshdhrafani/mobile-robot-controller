#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <cmath>

double kP=0.7;  //for omega
double kP_vel=0.7; //for forward velocity
double v0=1.0;
double alpha=0.7;

class hybrid_automata{
    private:
        double phi_desired;
        double goalDistance;
        double obstacleDistance;
        double obsAngle=0.0;
        double phi_cw,phi_ccw;

        bool checkfollowWallCCW()
        {
            phi_ccw = obsAngle - acos(0);
            phi_ccw = atan2(sin(phi_ccw),cos(phi_ccw));  //normalise to [-pi,pi]

            phi_cw = obsAngle + acos(0);
            phi_cw = atan2(sin(phi_cw),cos(phi_cw)); //normalise to [-pi,pi]
            
            //double attackAngle = phi_desired - phi_ccw;
            if(cos(phi_desired - phi_ccw)>0)
                return 1;
            else
            {
                //attackAngle = phi_desired - phi_cw;
                if(cos(phi_desired - phi_cw)>0)
                    return 0;
            }
            return 1;

        }

        double findObstacle(double ranges[])
        {   
            double min=100.0;
            for(int i=0;i<360;i++)
            {
                if(ranges[i]<min)
                {
                    min=ranges[i];
                    obsAngle=i;  //convert degrees to radians
                }
            }
            if(obsAngle>=0 && obsAngle<=180)
                obsAngle=obsAngle*2*acos(0)/180;
            else
                obsAngle=obsAngle*2*acos(0)/180 - 4*acos(0);
            return min;
        }

        double euclideanDistance(double x0, double y0, double x1, double y1){
            return sqrt(pow(x0-x1,2)+pow(y0-y1,2));
        }

        geometry_msgs::Twist stopBot()
        {
            geometry_msgs::Twist command_velo;
            command_velo.angular.x = 0;
            command_velo.angular.y = 0;
            command_velo.angular.z = 0; 

            command_velo.linear.x = 0;
            command_velo.linear.y = 0;
            command_velo.linear.z = 0;

            ROS_INFO("Goal Reached");

            return command_velo;
        }

        geometry_msgs::Twist obstacleAvoidance(double yaw_curr)
        {
            geometry_msgs::Twist command_velo;

            double headingError = obsAngle + acos(0) - yaw_curr;   //go opposite direction of obstacle
            headingError = atan2(sin(headingError),cos(headingError));

            ROS_INFO("Executing OA, obsAngle: %f",obsAngle);

            if(headingError>0.05)
            {
                command_velo.angular.x = 0;
                command_velo.angular.y = 0;
                command_velo.angular.z = 0.5*kP*headingError;
                command_velo.linear.x = 0;
                command_velo.linear.y = 0;
                command_velo.linear.z = 0;
                return command_velo;
            }
            else
            { 
                command_velo.angular.x = 0;
                command_velo.angular.y = 0;
                command_velo.angular.z = 0.5*kP*headingError;
                command_velo.linear.x = 0.3*kP_vel*v0;
                command_velo.linear.y = 0;
                command_velo.linear.z = 0;
                return command_velo;
            }
            
        }

        geometry_msgs::Twist slidingMode(double yaw_curr, bool ccw)
        {
            geometry_msgs::Twist command_velo;
            double headingError=0.0;
            
            //double attackAngle = obsAngle - phi_desired;
            if(cos(obsAngle-phi_desired+4*acos(0))>0)
            {
                return this->goToGoal(yaw_curr);
            }
            else
            {
                if(ccw)
                {
                    headingError = phi_ccw-yaw_curr;
                    headingError = atan2(sin(headingError),cos(headingError));              //normalise
                    ROS_INFO("Executing follow wall CCW, headingError: %f", headingError);
                }
                else
                {
                    headingError = phi_cw-yaw_curr;
                    headingError = atan2(sin(headingError),cos(headingError));               //normalise
                    ROS_INFO("Executing follow wall CW, headingError: %f", headingError);
                }

                kP=0.9;

                command_velo.angular.x = 0;
                command_velo.angular.y = 0;
                command_velo.angular.z = kP*headingError; 
                command_velo.linear.x = 0.5*kP_vel*v0; //for large error, v goes to v0
                command_velo.linear.y = 0;
                command_velo.linear.z = 0;

                return command_velo;
            }
        }

        geometry_msgs::Twist goToGoal(double yaw_curr)
        {
            geometry_msgs::Twist command_velo;
            double headingError = phi_desired-yaw_curr;
            headingError = atan2(sin(headingError),cos(headingError));

            ROS_INFO("Executing GTG, HeadingError: %f, DistToGoal: %f",headingError,goalDistance);

            command_velo.angular.x = 0;
            command_velo.angular.y = 0;
            command_velo.angular.z = kP*headingError; 

            command_velo.linear.x = kP_vel*v0*(1-exp(-1*alpha*pow(goalDistance,2))); //for large error, v goes to v0
            command_velo.linear.y = 0;
            command_velo.linear.z = 0;
            
            return command_velo;
        }
        
    public:
        geometry_msgs::Twist switcher(double x_curr, double y_curr, double yaw_curr, double x_desired, double y_desired, double range[])
        {
            phi_desired = atan2(y_desired-y_curr, x_desired-x_curr);
            goalDistance = this->euclideanDistance(x_curr,y_curr,x_desired,y_desired);
            obstacleDistance = this->findObstacle(range);
            if(goalDistance>0.05)
            {
                if(obstacleDistance>1.5)
                {
                    return this->goToGoal(yaw_curr);
                }
                else if(obstacleDistance>0.7)            //safe distance for sliding mode
                {
                    return this->slidingMode(yaw_curr,this->checkfollowWallCCW());
                }
                else       
                {
                    if(obsAngle<-0.523 || obsAngle>0.523)       //don't avoid obstacles if they are not in fov -30 to 30 deg
                    {
                        return this->goToGoal(yaw_curr);
                    }
                    else                //start pure recovery behaviour (obstacle avoidance)
                        return this->obstacleAvoidance(yaw_curr);
                }

            }
            else{
                return this->stopBot();
            }        
        }
};