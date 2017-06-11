#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int8.h"
#include <tf/transform_listener.h>
#include "geometry_msgs/Transform.h"
#include "gazebo_msgs/SetModelState.h"
#include <cmath>

#define VELOCITY 8

ros::Publisher motor_command_publisher;


tf::StampedTransform robot_pose;
std_msgs::Int8 msg1;
geometry_msgs::Twist motor_command;

double av;
int turn;
bool received = false;
void channelCallback(const std_msgs::Int8::ConstPtr& msg)
{
	turn=msg->data;
	received = true;
};

int main(int argc, char **argv)
{
	motor_command.linear.x=0;
	  
	ros::init(argc, argv, "turner_node");
	ros::NodeHandle n;

	float comp_x,comp_y;
	ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	tf::TransformListener listener;
	
	
	ros::Subscriber sub = n.subscribe("/motion/turn", 1000, channelCallback);
	
	
	//ros::Publisher pub = n.advertise<std_msgs::Int8>("/motion/act", 100);
	ros::Publisher pub = n.advertise<std_msgs::Int8>("/patroller/act", 100);
	
	geometry_msgs::Point pr2_position;
	pr2_position.x = 0.0;
	pr2_position.y = 0.0;
    pr2_position.z = 2.0;
   
    geometry_msgs::Quaternion pr2_orientation;
    pr2_orientation.x = 0.0;
    pr2_orientation.y = 0.0;
    pr2_orientation.z = 0.0;
    pr2_orientation.w = 1.0;


    geometry_msgs::Twist start_twist;
    start_twist.linear.x = 0;
    start_twist.linear.y = 0;
    start_twist.linear.z = 0.0;
    start_twist.angular.x = 0.0;
    start_twist.angular.y = 0.0;
    start_twist.angular.z = 0.0;
  
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 100);
 
    ros::Rate loop_rate(5);
  
    while (ros::ok())
    {
	  
	 
	    ros::spinOnce();
	  
	    
		//
	  
	    if (received)//Targeting and fire mode
	    {
		    std::cout << "received msg, turn: " << turn << std::endl;
		  
		    
		    //if(turn<=2&&turn>=-2)
		    //{//If the target is near to middle, fire free.
			  
				
		    //}
		   
				

				
				std::cout << "içteki turn: "<<turn << std::endl;
				av= -1.5 * ((double)turn/20);
				motor_command.angular.z = av;
				std::cout << "Angular velocity: "<<av << std::endl;
			    if (av > 0)
			    {
					std::cout << "Turn LEFT" << std::endl;
				}
				else if (av < 0)
				{
			        std::cout << "Turn RIGHT" << std::endl;
				}
			
			
		    motor_command_publisher.publish(motor_command);
	    	
	    	ros::Duration(1.0).sleep();
	    	
	    	
	    	try{
            //grab the latest available transform from the odometry frame (robot's original location - usually the same as the map unless the odometry becomes inaccurate) to the robot's base.
            listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), robot_pose);
        }
            //if something goes wrong with this just go to bed for a second or so and wake up hopefully refreshed.
            catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        
        double robot_x = robot_pose.getOrigin().x();
        double robot_y = robot_pose.getOrigin().y();
        pr2_position.x = robot_x;
		pr2_position.y = robot_y;
		pr2_position.z = 1.0;
        //double robot_z = robot_pose.getOrigin().z();

        tf::Vector3 robot_axis=robot_pose.getRotation().getAxis();
        double robot_theta=robot_pose.getRotation().getAngle()*robot_axis[2]; // only need the z axis
        //std::cout<<"Robot is believed to have orientation (theta): ("<<robot_theta<<")"<<std::endl<<std::endl;
		
		comp_x= VELOCITY * cos(robot_theta);
	    comp_y=VELOCITY * sin(robot_theta);
	    //std::cout << "comp_x:" << comp_x << std::endl;
	    //std::cout << 
	    start_twist.linear.x = comp_x;
		start_twist.linear.y = comp_y;
	    	
	    	std::cout<<"Bang Bang"<<std::endl;
				geometry_msgs::Pose pr2_pose;
				pr2_pose.position = pr2_position;
				pr2_pose.orientation = pr2_orientation;

				gazebo_msgs::ModelState pr2_modelstate;
				pr2_modelstate.model_name = (std::string) "wood_cube_10cm";
				pr2_modelstate.pose = pr2_pose;
				
				pr2_modelstate.twist=start_twist;
				std::cout<<"X component: "<<pr2_modelstate.twist.linear.x<<" Y component: "<<pr2_modelstate.twist.linear.y<<std::endl;
				
				
				
				gazebo_msgs::SetModelState srv;

	 
				srv.request.model_state = pr2_modelstate;
		    	if(client.call(srv))
		    	{
		    		ROS_INFO("PR2's magic moving success!!");
		    	}
		    	else
		    	{
		    		ROS_ERROR("Failed to magic move PR2! Error msg:%s",srv.response.status_message.c_str());
		    	}
		    	
		    	
	    	
	    	
			motor_command.angular.z=0;//Stop turning
			motor_command_publisher.publish(motor_command);
			
			
			received = false; //Close the tfm. ,
			msg1.data = 1;
			pub.publish(msg1); //Reactivate the patroller.
			std::cout << "activate patroller" << std::endl;
		}
		else
		{
			std::cout << "Waiting message to turn" << std::endl;
		}
			  
	    loop_rate.sleep();
    }
  

    return 0;
}
