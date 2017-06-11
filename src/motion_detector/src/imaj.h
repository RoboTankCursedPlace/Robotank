#ifndef IMAJ_H
#define IMAJ_H

#include <iostream> //  for easy output
#include <fstream>  //  for file output
#include <vector>   //  for ROS image msg data
#include <cmath>    //  for abs
#include <list>
#include <cstring>  //  for memset

#include "select.h"


#define RED_WEIGHT 0.2990
#define GREEN_WEIGHT 0.5870
#define BLUE_WEIGHT 0.1140


#define BUFFER_SIZE 21


#define BINARY_THRESHOLD 30


#define MOTION_THRESHOLD 10000

struct ConnectedComponent
{
	uint32_t area;
	
	//uint32_t leftX;
	//uint32_t leftY;
	
	//uint32_t middleX;
	//uint32_t middleY;
	
    //uint32_t rightX;
    //uint32_t rightY;
    
    uint32_t leftX;
    uint32_t rightX;
   
    uint32_t bottomY;
    uint32_t topY;
   
    uint32_t sumX;
    uint32_t sumY;
    
    uint32_t middleX;
    uint32_t middleY;
};

bool CompareConnectedComponent(const ConnectedComponent &c1, const ConnectedComponent &c2) {return c1.area < c2.area;}



//  Converts ROS RGB8 image to grayscale image
uint8_t** RGB2Gray(std::vector<uint8_t> &data, int rowCount, int columnCount);


////////////////////////////////////////////////
//                                            //
//   Functions that work with gray images     //
//                      (or single channel)   //
////////////////////////////////////////////////

//  Converts grayscale image to txt file pixel by pixel
void Image2Txt(uint8_t** data, int rowCount, int columnCount, const char* fileName);

//  Cheating function, not used of course
uint8_t** RemoveBackgroundCheat(uint8_t** data, int rowCount, int columnCount);

//  Gaussian smoothing with 11x11 kernel
uint8_t** GaussianSmooth(uint8_t** img, int rowCount, int columnCount);

//  Returns estimated background image
uint8_t** CreateBackgroundModel(std::list<uint8_t**> &buffer, int rowCount, int columnCount);

//  Returns difference between img1 and img2 (absolute values)
uint8_t** CalculateDifferenceImage(uint8_t** img1, uint8_t** img2, int rowCount, int columnCount);

//  Returns thresholded image
uint8_t** Convert2BinaryImage(uint8_t** img, int rowCount, int columnCount);

//  Returns filled image (if fourNeighbor is false, eightNeighbor is used)
uint8_t** Dilate(uint8_t** img, bool fourNeighbor, int rowCount, int columnCount);

//  Returns found connected components and puts them into given list
void FindConnectedComponents(uint8_t** img, int rowCount, int columnCount, std::vector<ConnectedComponent> &out);

#endif
