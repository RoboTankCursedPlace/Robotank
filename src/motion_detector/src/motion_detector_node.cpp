//  For ROS
#include <ros/ros.h>



//  For laserscan data
#include "sensor_msgs/LaserScan.h"

//  For camera data
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Image.h"

//  For communicating with other nodes
#include "std_msgs/Int8.h"


//  For console/file printing
#include <iostream>
#include <fstream>

//  For all neccesary math functions
#include <cmath>

//  ...
#include <vector>
#include <queue>
#include <list>


//  For everything
#include "imaj.h"


//  For sorting std::vector
#include <algorithm>


//  For timeout
#include <ctime>



#define TIMEOUT_SECONDS 10.0



////////////////////////////////////////////
//                                        //
//     Callback Functions and Data        //
//                                        //
////////////////////////////////////////////
bool currentlyProcessing = false;
bool hasPreviousFrame = false;
bool receivedActivation = false;

//  Callback data
sensor_msgs::LaserScan laser_msg;
sensor_msgs::Image image_msg;

//  Callback counters
unsigned long laser_callback_counter = 0;
unsigned long image_callback_counter = 0;

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	if (!receivedActivation)
	    return;
	    
	//std::cout << "laser_callback()" << std::endl;
	if (!currentlyProcessing)
	{
		laser_msg = *msg;
	}
	++laser_callback_counter;
	std::cout << "LaserScan - counter:" << laser_callback_counter << std::endl;
}

void image_callback(const sensor_msgs::Image::ConstPtr& msg)
{
	//std::cout << "image_callback()" << std::endl;
	if (!receivedActivation)
	    return;
	    
	if (!currentlyProcessing)
	{
		image_msg = *msg;
	}
	++image_callback_counter;
	
	if (receivedActivation)
	{
	    ////std::cout << "Image - counter:" << image_callback_counter << std::endl;
	    ;
	}
}

void activation_callback(const std_msgs::Int8::ConstPtr& msg)
{
	std::cout << "motion node activated!" << std::endl;
	receivedActivation = true;
}



////////////////////////////////////////////
//                                        //
//     Some printf for debugging          //
//                                        //
////////////////////////////////////////////
void PrintLaserData()
{
	std::cout << "LaserScan - counter:" << laser_callback_counter << std::endl;
    std::cout << "LaserScan - angle_min:" << laser_msg.angle_min << std::endl;
    std::cout << "LaserScan - angle_max:" << laser_msg.angle_max << std::endl;
}


void PrintImageData()
{
	std::cout << "Image - counter:" << image_callback_counter << std::endl;
	std::cout << "Image - height/rows:" << image_msg.height << std::endl;
	std::cout << "Image - width/columns:"  << image_msg.width  << std::endl;
	std::cout << "Image - encoding:" << image_msg.encoding << std::endl;
	std::cout << "Image - bigend?:" << image_msg.is_bigendian << std::endl;
	std::cout << "Image - step(row size?):" << image_msg.step << std::endl;
	std::cout << "Image - data length:" << image_msg.data.size() << std::endl;
}

//  Converts grayscale image to ROS img message
void Image2Msg( uint8_t** data, int rowCount, int columnCount, ros::Publisher &pubber)
{
	static const std::string grayEncoding = "mono8";
	
    sensor_msgs::Image imgmsg;
    
    imgmsg.header = image_msg.header;
    
    imgmsg.height = rowCount;
    imgmsg.width = columnCount;
	
	imgmsg.encoding = grayEncoding;
    imgmsg.is_bigendian = image_msg.is_bigendian;
    imgmsg.step = columnCount;
    
    //imgmsg.data.assign(data, data + 3);
    //imgmsg.data.reserve(rowCount * columnCount);
    //memcpy(&imgmsg.data, data, rowCount*columnCount*8);
    //std::copy(data, data + rowCount*columnCount, imgmsg.data.begin());
    
    //imgmsg.data.resize(imgmsg.data.size() + rowCount*columnCount);
    //memcpy(&imgmsg.data[imgmsg.data.size() - rowCount*columnCount], &data[0][0], rowCount*columnCount * sizeof(uint8_t));
    
    
    
    
    imgmsg.data.reserve(rowCount*columnCount);
    for (int i = 0; i < rowCount; ++i)
    {
		for (int j = 0; j < columnCount; ++j)
		{
			imgmsg.data.push_back(data[i][j]);
		}
	}
	
	
	
	//imgmsg.data.reserve(rowCount*columnCount);
	
	//std::cout << "inserting\n";
	//imgmsg.data.insert(imgmsg.data.end(), &data[0][0], &data[0][0] + rowCount*columnCount);
	//imgmsg.data.resize(rowCount*columnCount);
	//memcpy(&imgmsg.data.front(), &data[0][0], rowCount*columnCount);
	//std::cout << "vector size:" << imgmsg.data.size() << std::endl;
	//std::cout << "inserted\n";
	
	/*
	imgmsg.data.reserve(rowCount*columnCount);
	//imgmsg.data.insert(imgmsg.data.end(), &data[0][0], &data[rowCount-1][columnCount-1]);
    imgmsg.data.assign(&data[0][0], &data[0][0] + rowCount*columnCount);
    */
    
    
    //imgmsg.data.resize(rowCount*columnCount);
    //memcpy(&imgmsg.data[0], &data[0][0], rowCount*columnCount);
    
    
    pubber.publish(imgmsg);
    
}

////////////////////////////////////////////
//                                        //
//     Image Processing Functions         //
//                                        //
////////////////////////////////////////////
#define APPLY_GAUSSIAN_SMOOTH false

std::list<uint8_t**> buffer;


//uint8_t** backgroundModel = NULL;

uint8_t** grayImagePrev = NULL;
uint8_t** filteredImagePrev = NULL;


int main(int argc, char* argv[])
{
	//  This must be called before anything else ROS-related
	ros::init(argc, argv, "motion_detector_node");
	
	//  Create a ROS node handle
	ros::NodeHandle node;
	
	//  Subscribe to all important sensor messages
	ros::Subscriber camera_subscriber = node.subscribe("/camera/rgb/image_raw", 1000, image_callback);
	ros::Subscriber activation_subscriber = node.subscribe("/motion/act", 1000, activation_callback);
	//ros::Subscriber turner_subscriber = node.subscribe("/motion/stop", 1000, turner_callback);
	
	
	//ros::Subscriber laser_subscriber = node.subscribe("/scan", 1000, laser_callback);
	ros::Publisher turner_publisher = node.advertise<std_msgs::Int8>("/motion/turn", 100);
	ros::Publisher patroller_publisher = node.advertise<std_msgs::Int8>("/patroller/act", 100);
	ros::Publisher rviz_current_publisher = node.advertise<sensor_msgs::Image>("motion/currentImg", 10);
	ros::Publisher rviz_background_publisher = node.advertise<sensor_msgs::Image>("motion/backgroundImg", 10);
	ros::Publisher rviz_final_publisher = node.advertise<sensor_msgs::Image>("motion/finalImage", 10);
	
	
	//  Set the rate at which we print out our message (1Hz)
	ros::Rate loop_rate(1.0);
	ros::Rate wait_rate(1.0);
	//ros::Rate background_loop_rate(10.0);
	
	//  Counters of callback functions
	int prev_laser_counter = 0;
	int prev_image_counter = 0;
	
	
	//  Wait until background buffer is full
	initbackground:
	
	while (!receivedActivation)
	{
		//  Wait until motion detector node is activated by patroller
		 ros::spinOnce(); //  magic???
		 
		 std::cout << "Motion Detector - WAITING FOR ACTIVATION" << std::endl;
		 
		 wait_rate.sleep();
		 
		continue;
    }
	
	std::cout << "Filling initial background buffer" << std::endl;
	while (buffer.size() < BUFFER_SIZE)
	{
		ros::spinOnce(); //  magic???
		
		//  Wait until we received a new image message
		if (image_callback_counter != prev_image_counter)
		{
			prev_image_counter = image_callback_counter;
			uint8_t** grayImage = RGB2Gray(image_msg.data, image_msg.height, image_msg.width);
			
			if (APPLY_GAUSSIAN_SMOOTH)
			{
				uint8_t** filteredImage = GaussianSmooth(grayImage, image_msg.height, image_msg.width);
				for (int row = 0; row < image_msg.height; ++row)
				{
					delete[] grayImage[row];
				}
				
				buffer.push_back(filteredImage);
			}
			else
			{
			    buffer.push_back(grayImage);
			}

		}
		
		//  Wait the stated duration
		//background_loop_rate.sleep();
	}
	std::cout << "Buffer is ready" << std::endl;
	
	//  Initialize background model
	//std::cout << "Initialize background model" << std::endl;
	
	
	
    std::vector<ConnectedComponent> components;
    
    
    
    time_t startTime;
    time(&startTime);
	
	//  Loop through until the ROS system tells the user to shut down
	while (ros::ok())
	{
        ros::spinOnce(); //  magic???
		
		
		time_t currentTime;
		time(&currentTime);
		
		double elapsedTime = difftime(currentTime, startTime);
		if (elapsedTime >= TIMEOUT_SECONDS)
		{
			//  There is nothing to detect, reactivate patroller node
	        std_msgs::Int8 patrolmsg;
				  
		    patrolmsg.data = 2;		  
			patroller_publisher.publish(patrolmsg);
				    
				    
			receivedActivation = false;
				    
			std::cout << "NO MOTION TO DETECT - Deactivating motion detector!" << std::endl;
				    
				    
			//  Clear entire background buffer and wait for activation
		 	for (std::list<uint8_t**>::const_iterator it = buffer.begin(); it != buffer.end(); ++it)
		    {
			    uint8_t** frameT = *it;
			    for (int row = 0; row < image_msg.height; ++row)
		        {
					delete[] frameT[row];
				}
			}
			buffer.clear();
			currentlyProcessing = false;
				    
		    goto initbackground;
		
	    }
		
		std::cout << std::endl;
		std::cout << "Current frame:" << image_callback_counter << std::endl;
		
		
		//  Wait until a new frame is received
		if (image_callback_counter != prev_image_counter)
		{
			prev_image_counter = image_callback_counter;
			
			currentlyProcessing = true;
			std::cout << "---Processing started---" << std::endl;
		
			uint32_t rowCount = image_msg.height;
			uint32_t columnCount = image_msg.width;
			uint32_t dataLength = rowCount * columnCount * 3; //  *3 because RGB channels
			
			
			//  Get current frame
			uint8_t** currentFrame = RGB2Gray(image_msg.data, rowCount, columnCount);
			//std::cout << "grayed\n";
			
			
			//  This step is neccesary if Duff adds some noise to the camera input
			if (APPLY_GAUSSIAN_SMOOTH)
			{
			    //  Gaussian smooth
		    	
		    	uint8_t** filteredImage = GaussianSmooth(currentFrame, rowCount, columnCount);
		    	//std::cout << "gaus1\n";
		    	for (int row = 0; row < rowCount; ++row)
		    	{
					delete[] currentFrame[row];
					
				}
				//std::cout << "gaus2\n";
				
				
				for (int row = 0; row < rowCount; ++row)
				{
					currentFrame[row] = filteredImage[row];
				}
				//std::cout << "gaus3\n";
				currentFrame = filteredImage;
			} 
			//std::cout << "filtered\n";
			
			//  Create background model
			uint8_t** backgroundModel = CreateBackgroundModel(buffer, rowCount, columnCount);
			//std::cout << "bged\n";
			
			//  Calculate difference between background and current frame
			uint8_t** delta = CalculateDifferenceImage(currentFrame, backgroundModel, rowCount, columnCount);
			//std::cout << "diffed\n";
			
			//  Convert delta image to binary image (values 0 and 255 only)
			uint8_t** binaryImage = Convert2BinaryImage(delta, rowCount, columnCount);
			//std::cout << "binaried\n";
			
			
			//  Fill the holes (how much important?)
			uint8_t** filledImage = Dilate(binaryImage, false, rowCount, columnCount);
			//std::cout << "dilated\n";
			
			//  Find connected components
			components.reserve(255);
			FindConnectedComponents(filledImage, rowCount, columnCount, components);
			//std::cout << "found components\n";
			
			//  Print info of components
			//std::cout << "Number of components:" << components.size() << std::endl;
			
			if (components.size() > 0)
			{
			    std::sort(components.begin(), components.end(), CompareConnectedComponent);
			    int lastIndex = components.size() - 1;
			    if (components[lastIndex].area >= MOTION_THRESHOLD)
		    	{
		    		std::cout << "Largest component area:" << components[lastIndex].area << std::endl;
		    		
		    		
		    		/*
		    		Image2Txt(currentFrame, rowCount, columnCount, "grayimage.txt");
		    		//Image2Txt(filteredImage, rowCount, columnCount, "filteredimage.txt");
		    		Image2Txt(backgroundModel, rowCount, columnCount, "background.txt");
		    		Image2Txt(delta, rowCount, columnCount, "deltaimage.txt");
		    		Image2Txt(binaryImage, rowCount, columnCount, "binaryimage.txt");
		    		Image2Txt(filledImage, rowCount, columnCount, "1xdilatedImage.txt");
				*/
		    		//  Print component info to a file
		    		std::ofstream file("largestcomponent.txt");
		    		file << "Area:" << components[lastIndex].area << std::endl;
		    		file << "leftX:" << components[lastIndex].leftX << std::endl;
		    		file << "rightX:" << components[lastIndex].rightX << std::endl;
		    		file << "topY:" << components[lastIndex].topY << std::endl;
		    		file << "bottomY:" << components[lastIndex].bottomY << std::endl;
		    		file << "middleX:" << components[lastIndex].middleX << std::endl;
		    		file << "middleY:" << components[lastIndex].middleY << std::endl;
		    		file.close();
				
				    
				    
				    std_msgs::Int8 turnmsg;
				    
				    int partitionSize = columnCount / 40;
				    int sector = components[lastIndex].middleX / partitionSize;
				    
				    sector = sector - 20;
				    
				    turnmsg.data = sector;
				    turner_publisher.publish(turnmsg);
				    
				    
				    receivedActivation = false;
				    
				    std::cout << "Deactivating motion detector!" << std::endl;
				    
				    
				    //  Clear entire background buffer and wait for activation
		 		    for (std::list<uint8_t**>::const_iterator it = buffer.begin(); it != buffer.end(); ++it)
		        	{
			        	uint8_t** frameT = *it;
			        	for (int row = 0; row < rowCount; ++row)
						{
							delete[] frameT[row];
						}
			        }
			        buffer.clear();
			        components.clear();
			        currentlyProcessing = false;
				    
				    goto initbackground;
			    }
			    else
			    {
					std::cout << "NO MOTION DETECTED" << std::endl;
				}
			}
			else
			{
				std::cout << "NO MOTION DETECTED()" << std::endl;
			}
			
			
			//Image2Msg(currentFrame, rowCount, columnCount, rviz_current_publisher);
			//Image2Msg(backgroundModel, rowCount, columnCount, rviz_background_publisher);
			Image2Msg(filledImage, rowCount, columnCount, rviz_final_publisher);
			
			
			
			
			//-  Remove first frame from buffer
			//-  Deallocate first frame from memory
			//-  Add current frame to last position of buffer
			uint8_t** firstFrame = buffer.front();
			buffer.pop_front();
			for (int row = 0; row < rowCount; ++row)
			{
				delete[] firstFrame[row];
			}
			//delete[] firstFrame;
			buffer.push_back(currentFrame);
			std::cout << "Removed and added first frame\n";
			
			
			//  Cleanup
			for (int row = 0; row < rowCount; ++row)
			{
				delete[] backgroundModel[row];
				//std::cout << "--bg\n";
				
				delete[] delta[row];
				//std::cout << "--dl\n";
				
				delete[] binaryImage[row];
				//std::cout << "--bi\n";
				
				delete[] filledImage[row];
				//std::cout << "--fi\n";
			}
			//delete[] backgroundModel;
			//delete[] delta;
			//delete[] binaryImage;
			//delete[] filledImage;
			//std::cout << "Deallocated images\n";
			
			components.clear();
			//std::cout << "Deallocated components\n";
			
			currentlyProcessing = false;
			
			//std::cout << "---Processing Finished---" << std::endl;
		}
		
		//  Wait the stated duration
		//std::cout << "ooo\n";
		loop_rate.sleep();
	}
	
	
	//  Exit the program.
	return 0;
}
