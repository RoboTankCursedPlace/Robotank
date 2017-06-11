//  For ROS
#include "ros/ros.h"


//  For navigating around
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include <tf/transform_listener.h>


//  For communicating between other nodes
#include "std_msgs/Int8.h"


//  For I/O
#include <iostream>
#include <fstream>


#include <vector>
#include <iterator>


//  Math
#include <cmath>

#include "grid.h"
Grid *map;



//  Frequency of main while loop
const double LoopFrequency = 2.0;
const double LoopTime = 1/LoopFrequency;

//taken from referee.py file
const double COSTHETA_CLOSENESS_THRESH = 0.5 - 0.5*cos(M_PI/4);
const double XY_CLOSENESS_THRESH = 0.20;

const bool TRY_DOR = true;


const bool HOCA_DEBUG_MSG = true;


//  For getting robot position
tf::StampedTransform robot_pose;


//  For sending motor commands
geometry_msgs::Twist motor_command;
ros::Publisher motor_command_publisher;


bool receivedActivation = true;


void activation_callback(const std_msgs::Int8::ConstPtr& msg)
{
	std::cout << "patroller node activated!" << std::endl;
	receivedActivation = true;
}

    
void MoveRobot(std::list<geometry_msgs::Point> &path, double finalRotation)
{
	int robotRow;
	int robotColumn;
	
	tf::TransformListener listener;
	
	
	geometry_msgs::Point waypoint = path.front();
	
    ros::Rate delay(LoopFrequency); // perhaps this could be faster for a controller?
    while(ros::ok()){

        //***************************************
        //***          Obtain current robot pose
        //***************************************

        ros::spinOnce(); // may be needed to call the callback
        ros::spinOnce(); // may be needed to call the callback

        try{
            //grab the latest available transform from the odometry frame (robot's original location - usually the same as the map unless the odometry becomes inaccurate) to the robot's base.
            listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), robot_pose);
        }
            //if something goes wrong with this just go to bed for a second or so and wake up hopefully refreshed.
            catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        //***************************************
        //***          Print current robot pose
        //***************************************

        //Print out the x,y coordinates of the transform
        if (HOCA_DEBUG_MSG) std::cout<<"Robot is believed to be at (x,y): ("<<robot_pose.getOrigin().x()<<","<<robot_pose.getOrigin().y()<<")"<<std::endl;

        //Convert the quaternion-based orientation of the latest message to angle-axis in order to get the z rotation & print it.
        tf::Vector3 robot_axis=robot_pose.getRotation().getAxis();
        double robot_theta=robot_pose.getRotation().getAngle()*robot_axis[2]; // only need the z axis
        if (HOCA_DEBUG_MSG) std::cout<<"Robot is believed to have orientation (theta): ("<<robot_theta<<")"<<std::endl<<std::endl;

        //***************************************
        //***          Print current destination
        //***************************************

        // the curr_waypoint variable is filled in in the waypoint_callback function above, which comes from incoming messages
        // subscribed to in the .subscribe function call above.

        //Print out the x,y coordinates of the latest message
        if (HOCA_DEBUG_MSG) std::cout<<"Current waypoint (x,y): ("<<waypoint.x<<","<<waypoint.y<<")"<<std::endl;

        /*
        //Convert the quaternion-based orientation of the latest message to angle-axis in order to get the z rotation & print it.
        tf::Quaternion quat(waypoint.rotation.x,waypoint.rotation.y,waypoint.rotation.z,waypoint.rotation.w);
        tf::Vector3 waypoint_axis=quat.getAxis();
        double waypoint_theta=quat.getAngle()*waypoint_axis[2]; // only need the z axis
        if (HOCA_DEBUG_MSG) std::cout<<"Current waypoint (theta): ("<<waypoint_theta<<")"<<std::endl<<std::endl;
        */
        double waypoint_theta = 0;
        //  make waypoint_theta point to next point
        if (path.size() > 1)
        {
		    //geometry_msgs::Point nextWaypoint = path.begin()
		    
		    std::list<geometry_msgs::Point>::iterator nextWaypoint = path.begin();
		    ++nextWaypoint;
		    
		    waypoint_theta = AngleBetweenPoints(waypoint.x, waypoint.y, nextWaypoint->x, nextWaypoint->y);
		}
		else
		{
			waypoint_theta = finalRotation;
		}
        
        
        //***************************************
        //***          DRIVE THE ROBOT HERE (same as with assignment 1)
        //***************************************
        
        //  Create some variables for easier reading
        double robotX = robot_pose.getOrigin().x();
        double robotY = robot_pose.getOrigin().y();
        double waypointX = waypoint.x;
        double waypointY = waypoint.y;
        
        
        //  Convert real coordinates of waypoint to robot's frame
        tf::Vector3 pointWaypoint(waypointX, waypointY, 0);
        tf::Vector3 pointR = robot_pose.inverse() * pointWaypoint;
        double x = pointR.getX();
        double y = pointR.getY();
        
        
        //  Calculate angle from robot to waypoint
        double turning_angle = AngleBetweenPoints(0, 0, x, y);
        std::cout << "Robot needs to turn " << turning_angle << " radians" << std::endl;
       
       
        //  Calculate distance from robot to waypoint
        double distanceX = fabs(x - 0);
        double distanceY = fabs(y - 0);
        double distance = DistanceBetweenPoints(0, 0, x, y); //sqrt(distanceX*distanceX + distanceY*distanceY)
        std::cout << "Robot is " << distance << " meters(?) away from the waypoint" << std::endl;
        
        if (distance <= XY_CLOSENESS_THRESH)
        {
			path.pop_front();
			
			if (path.size() == 0)
			{
				break;
			}
			else
			{
			    waypoint = path.front();
		    	continue;
			}
		}
       
 
        //  Decide angular speed
        double angularSpeed;
        
        if (TRY_DOR && distance < XY_CLOSENESS_THRESH)
        {
			//  Fix negative theta values
			if (robot_theta < 0) robot_theta = 2*M_PI + robot_theta;
			if (waypoint_theta < 0) waypoint_theta = 2*M_PI + waypoint_theta;
			
			
			if (fabs(robot_theta - waypoint_theta) <= COSTHETA_CLOSENESS_THRESH)
			{
				angularSpeed = 0.0;
			}
			else
			{
			    angularSpeed = waypoint_theta - robot_theta;
			    
			    //  Prefer short turn over long one
			    if (fabs(angularSpeed) > M_PI)
			    {
					if (angularSpeed > 0)
					{
						angularSpeed = 2*M_PI - angularSpeed;  //  convert 200 to 160
					}
					else
					{
						angularSpeed = -2*M_PI - angularSpeed; //  convert -250 to -110
			        }
				}
			}
		}
		else
		{
			if (fabs(turning_angle) <= COSTHETA_CLOSENESS_THRESH)
		    {
			    angularSpeed = 0.0;
		    }
		    else
		    {
		    	angularSpeed = (turning_angle / LoopFrequency);
		    }
		} 
        motor_command.angular.z = angularSpeed;
        std::cout << "Angular speed:" << angularSpeed << " radians" << std::endl;
        
        
        
        //  Decide linear speed
        double linearSpeed;
        if (x <= 0)
        {
			linearSpeed = (x / LoopFrequency) - 0.01;;
		}
		else
		{
			linearSpeed = (x / LoopFrequency) + 0.01;
		}
		
		
        motor_command.linear.x = linearSpeed;
        std::cout << "Linear speed:" << linearSpeed << " meters(?)" << std::endl;
        
        
        //  Publish twist msg
        motor_command_publisher.publish(motor_command);
        
		std::cout << std::endl;
        delay.sleep();
    }	
}


int main(int argc, char* argv[])
{
	
	ros::init(argc, argv, "patroller_node");
	
	ros::NodeHandle n;
	
	
	//  Subscribe to reactivation messages
	ros::Subscriber activation_subscriber = n.subscribe("/patroller/act", 1000, activation_callback);
	
	//  Publish activation message for motion detector node
	ros::Publisher pub = n.advertise<std_msgs::Int8>("/motion/act", 100);
	
	//  Publish motor commands to move robot around
	motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 100);
	
	
	
	std::cout << "Starting patroller node" << std::endl;
	
	map = new Grid("robotankMAP.txt");
	
	
	
	
	tf::TransformListener listener;
	
	
	int patrolRows[] = {7, 5, 15};
	int patrolColumns[] = {8, 17, 33};
	double patrolRotations[] = {-M_PI/4, -M_PI/4, -M_PI/2};
	int currentPatrol = 0;
	
	
	//  Wait until activated
	waitForActivation:
	while ( !receivedActivation )
	{
		ros::spinOnce(); //  magic???
	}
	
	
	ros::Duration(1.0).sleep();
	
	//  Wait for callbacks
	ros::spinOnce();	
	
	
	
	
	
	//  Get current patrol point
	std::cout << "Patrolling index:" << currentPatrol << std::endl;
	int goalRow = patrolRows[currentPatrol];
	int goalColumn = patrolColumns[currentPatrol];
	double patrolRotation = patrolRotations[currentPatrol];
	
	
	
	std::list<geometry_msgs::Point> pts;
	try
	{
       //grab the latest available transform from the odometry frame (robot's original location - usually the same as the map unless the odometry becomes inaccurate) to the robot's base.
       listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), robot_pose);
    }
       //if something goes wrong with this just go to bed for a second or so and wake up hopefully refreshed.
    catch (tf::TransformException ex)
    {
       ROS_ERROR("%s",ex.what());
       ros::Duration(1.0).sleep();
    }
	
	
	//  Find a path
	geometry_msgs::Point robotPoint;
	robotPoint.x = robot_pose.getOrigin().x();
	robotPoint.y = robot_pose.getOrigin().y();
	robotPoint.z = 0;
	int r, c;
	map->Point2Grid(r, c, robotPoint);
	
	map->AStar(r, c, goalRow, goalColumn, pts);
	std::cout << "Printing path:" << std::endl;
	map->PrintPath(pts);
	
	//  Extend the path
	std::cout << "Printing Extended Path" << std::endl;
	std::list<geometry_msgs::Point> ptsExtended;
	map->ExtendPath(pts, ptsExtended);
	map->PrintPath(ptsExtended);
	
	
	MoveRobot(ptsExtended, patrolRotation);
	
	
	//  Switch to next patrol point
	++currentPatrol;
	if (currentPatrol > 2)
	{
		currentPatrol = 0;
	}
	
	//  Deactivate itself and activate motion detector
	receivedActivation = false;
	std_msgs::Int8 msg1;
	msg1.data = 1;
	pub.publish(msg1); //  Activate motion detector
	goto waitForActivation;
	
	
	
	return 0;
}
