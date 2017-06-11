#include "imaj.h"


uint8_t** RGB2Gray(std::vector<uint8_t> &data, int rowCount, int columnCount)
{
	//static const double redWeight = 0.2990;
	//static const double greenWeight = 0.5870;
	//static const double blueWeight = 0.1140;
	
	uint32_t dataLength = rowCount * columnCount * 3; // *3 because RGB channels
	
	//  Initialize gray image
	uint8_t** grayImage = new uint8_t*[rowCount];
	for (int row = 0; row < rowCount; ++row)
	{
		grayImage[row] = new uint8_t[columnCount];
		/*
		for (int column = 0; column < columnCount; ++column)
		{
			grayImage[row][column] = 0;
		}
		*/
	}
	
	
	//  Convert RGB image to gray image
	for (int row = 0; row < rowCount; ++row)
	{
		
		for (int column = 0; column < columnCount; ++column)
		{
			int pixelIndex = row*columnCount + column;
			int dataIndex = pixelIndex * 3;
			double red = (double) data[dataIndex] * RED_WEIGHT;
			double green = (double) data[dataIndex + 1] * GREEN_WEIGHT;
			double blue = (double) data[dataIndex + 2] * BLUE_WEIGHT;
			uint8_t gray = (red + green + blue);
			//std::cout << (int)gray << std::endl;
		    grayImage[row][column] = gray;
		}
	}
			
	return grayImage;
}



void Image2Txt(uint8_t** data, int rowCount, int columnCount, const char* fileName)
{
	std::ofstream file(fileName);
	for (int row = 0; row < rowCount; ++row)
	{
		for (int column = 0; column < columnCount; ++column)
		{
			file << (int)data[row][column] << '\t';
		}
		file << std::endl;
	}
	file.close();
}




uint8_t** RemoveBackgroundCheat(uint8_t** grayImage, int rowCount, int columnCount)
{
	static const uint8_t backgrounds[] = {155, 178, 79};
	static const uint8_t errors[] = {5, 5, 5};
	
	static const int backgroundCount = 3;
	
	
	//  Initialize filtered image
	uint8_t** filteredImage = new uint8_t*[rowCount];
	for (int row = 0; row < rowCount; ++row)
	{
	    filteredImage[row] = new uint8_t[columnCount];
	    for (int column = 0; column < columnCount; ++column)
	    {
			filteredImage[row][column] = 0;
		}
	}
	
	
    //  Filter out background from image
    for (int row = 0; row < rowCount; ++row)
    {
		for (int column = 0; column < columnCount; ++column)
		{
			uint8_t pixelIntensity = grayImage[row][column];
			
			if (pixelIntensity <= 10)
			{
				pixelIntensity = 1;
			}
			else
			{
			    //  actual filtering
		    	for (int i = 0; i < backgroundCount; ++i)
		    	{
		    		if ( std::abs(pixelIntensity - backgrounds[i]) <= errors[i] )
			    	{
				    	pixelIntensity = 0;
				    	break;
					}
				}
			}
			
			std::cout << (int)pixelIntensity << std::endl;
			filteredImage[row][column] = pixelIntensity;
		}
	}
	
	return filteredImage;
}

uint8_t** GaussianSmooth(uint8_t** img, int rowCount, int columnCount)
{
    static const uint8_t kernel[11][11] = { {1, 1, 1, 2, 2, 2, 2, 2, 1, 1, 1},
		                      {1, 2, 2, 3, 4, 4, 4, 3, 2, 2, 1},
		                      {1, 2, 4, 5, 6, 7, 6, 5, 4, 2, 1},
		                      {2, 3, 5, 7, 8, 9, 8, 7, 5, 3, 2},
		                      {2, 4, 6, 8, 10, 11, 10, 8, 6, 4, 2},
		                      {2, 4, 7, 9, 11, 12, 11, 9, 7, 4, 2},
		                      {2, 4, 6, 8, 10, 11, 10, 8, 6, 4, 2},
		                      {2, 3, 5, 7, 8, 9, 8, 7, 5, 3, 2},
		                      {1, 2, 4, 5, 6, 7, 6, 5, 4, 2, 1},
		                      {1, 2, 2, 3, 4, 4, 4, 3, 2, 2, 1},
		                      {1, 1, 1, 2, 2, 2, 2, 2, 1, 1, 1} };
    static const uint32_t weight = 512;
    
    //  Initialize filtered image
	uint8_t** filteredImage = new uint8_t*[rowCount];
	for (int row = 0; row < rowCount; ++row)
	{
	    filteredImage[row] = new uint8_t[columnCount];
	    for (int column = 0; column < columnCount; ++column)
	    {
			filteredImage[row][column] = 0;
		}
	}
	
	
	//  Apply gaussian kernel 
	for (int row = 5; row < rowCount - 5; ++row)
	{
		for (int column = 5; column < columnCount - 5; ++column)
		{
			uint32_t sum = 0;
			for (int i = -5; i <= 5; ++i)
			{
				for (int j = -5; j <= 5; ++j)
				{
					sum += img[row + i][column + j] * kernel[i+5][j+5];
				}
			}
			filteredImage[row][column] = sum / weight;
		}
	}
	
	return filteredImage;
}

uint8_t** CreateBackgroundModel(std::list<uint8_t**> &buffer, int rowCount, int columnCount)
{
	uint8_t arr[BUFFER_SIZE];
	
	//  Initialize background
	uint8_t** backgroundModel = new uint8_t*[rowCount];
	for (int row = 0; row < rowCount; ++row)
	{
		backgroundModel[row] = new uint8_t[columnCount];
	}
	
	
	
	
	//  Fill background
	for (int row = 0; row < rowCount; ++row)
	{
		for (int column = 0; column < columnCount; ++column)
		{
			int i = 0;
			for (std::list<uint8_t**>::const_iterator it = buffer.begin(); it != buffer.end(); ++it)
			{
				uint8_t** frameT = *it;
				arr[i] = frameT[row][column];
				++i;
			}
				
			
			/*
			//  Iterate each frame
			for (int i = 0; i < BUFFER_SIZE; ++i)
			{
				arr[i] = buffer[i][row][column];
			}
			* */
			
			
			//  Get the median pixel
			uint8_t median = QuickSelectRecursive(arr, 0, BUFFER_SIZE-1, BUFFER_SIZE/2);
			
			
			//std::cout << "Background modeling - found median:" << (int)median << std::endl;
			
			backgroundModel[row][column] = median;
			
		}
	}
	
	return backgroundModel;
}

//  Returns difference between img1 and img2 (absolute values)
uint8_t** CalculateDifferenceImage(uint8_t** img1, uint8_t** img2, int rowCount, int columnCount)
{
	uint8_t** differenceImage = new uint8_t*[rowCount];
	for (int row = 0; row < rowCount; ++row)
	{
		differenceImage[row] = new uint8_t[columnCount];
		
		for (int column = 0; column < columnCount; ++column)
		{
		    differenceImage[row][column] = std::abs(img1[row][column] - img2[row][column]);
		}
	}
	
	return differenceImage;
}



uint8_t** Convert2BinaryImage(uint8_t** img, int rowCount, int columnCount)
{
	uint8_t** binaryImage = new uint8_t*[rowCount];
	for (int row = 0; row < rowCount; ++row)
	{
		binaryImage[row] = new uint8_t[columnCount];
		
		for (int column = 0; column < columnCount; ++column)
		{
		    if (img[row][column] >= BINARY_THRESHOLD)
	    	{
	    		binaryImage[row][column] = 255;
	    	}
	    	else
	    	{
		    	binaryImage[row][column] = 0;
			}
		}
	}
	
	return binaryImage;
}


uint8_t** Dilate(uint8_t** img, bool fourNeighbor, int rowCount, int columnCount)
{
	uint8_t** dilatedImage = new uint8_t*[rowCount];
    for (int row = 0; row < rowCount; ++row)
    {
		dilatedImage[row] = new uint8_t[columnCount];
	}

	
	//  Apply a 3x3 kernel
	//static const uint8_t kernel_4[3][3] = { {0, 1, 0}, {1, 1, 1}, {0, 1, 0}};
	//static const uint8_t kernel_8[3][3] = { {1, 1, 1}, {1, 1, 1}, {1, 1, 1}};
	
	
	//  Clear first and last row of the image
	for (int column = 0; column < columnCount; ++column)
	{
		dilatedImage[0][column] = 0;
		dilatedImage[rowCount-1][column] = 0;
	}
	
	//  Clear first and last column of the image
	for (int row = 0; row < rowCount; ++row)
	{
		dilatedImage[row][0] = 0;
		dilatedImage[row][columnCount-1] = 0;
	}
	
	
	for (int row = 1; row < rowCount - 1; ++row)
	{
		for (int column = 1; column < columnCount - 1; ++column)
		{
			
			//  No kernel, just do it manually
			int sum = 0;
			
			
			if (!fourNeighbor)
			{
				// 8 neighbor
			    for (int i = -1; i <= 1; ++i)
		    	{
			    	for (int j = -1; j <= 1; ++j)
			    	{
			    		sum += img[row + i][column + j];
			    	}
		    	}
			}
			else
			{
				// 4 neighbor
				sum += img[row-1][column] + img[row][column-1] + img[row][column] + img[row][column + 1] + img[row + 1][column];
			}
			
			
			if (sum > 0)
			    dilatedImage[row][column] = 255;
			else
			    dilatedImage[row][column] = 0;
			
		}
	}
	
	return dilatedImage;
}
    
    
void FindConnectedComponents(uint8_t** img, int rowCount, int columnCount, std::vector<ConnectedComponent> &components)
{
	static ConnectedComponent component;
	memset(&component, 0, sizeof(ConnectedComponent));
	component.leftX = 0x00ffffff;
	component.topY = 0x00ffffff;
	
	//  Initialize marker matrix
	uint8_t** marks = new uint8_t*[rowCount];
	for (int row = 0; row < rowCount; ++row)
	{
		marks[row] = new uint8_t[columnCount];
		for (int column = 0; column < columnCount; ++column)
		{
		    marks[row][column] = 255;
		}
	}
	
    uint8_t currentMark = 0;
    for (int row = 1; row < rowCount - 1; ++row)
    {
		for (int column = 1; column < columnCount - 1; ++column)
		{
			//std::cout << "[" << row << "][" << column << "],  " << (int)currentMark << "\n";
			
			
			//  Check if the current pixel is foreground
			if (img[row][column])
			{
				//  Check if there is a marked neighbor pixel (4 neighbor)
				uint8_t pixelMark = 255;
				for (int i = 0; i < currentMark; ++i)
				{
					if ( (marks[row-1][column] == i) || (marks[row][column-1] == i) || (marks[row][column+1]) == i || (marks[row+1][column] == i) )
					{
						pixelMark = i;
						break;
					}
				}
				
				
				//  If there is not a marked neighbor pixel, this is a new connected component
				if (pixelMark == 255)
				{
					////std::cout << "adding comp\n";
					pixelMark = currentMark;
					components.push_back(component);
					++currentMark;
					////std::cout << "Pixel mark:" << (int) pixelMark << std::endl;
					////std::cout << "last available mark:" << (int) currentMark << std::endl;
					////std::cout << "added comp\n";
				}
				
				
				//  Update component info
				components[pixelMark].area = components[pixelMark].area + 1;
				
				if (column < components[pixelMark].leftX)
				    components[pixelMark].leftX = column;
				
				if (column > components[pixelMark].rightX)
				    components[pixelMark].rightX = column;
				   
				if (row < components[pixelMark].topY)
				    components[pixelMark].topY = row;
				
				if (row > components[pixelMark].bottomY)
				    components[pixelMark].bottomY = row;
				
				components[pixelMark].sumX = components[pixelMark].sumX + column;
				components[pixelMark].sumY = components[pixelMark].sumY + row;
				
				
				//  Mark the current pixel
				marks[row][column] = pixelMark;
			} 
		    else
		    {
				//  Skip
				;
			}
		}
	}
	
	
	//  Calculate middle coordinates of components
	////std::cout << "cc1\n";
	for (int i = 0; i < components.size(); ++i)
	{
		if (components[i].area == 0)
		{
			components[i].middleX = 0;
			components[i].middleY = 0;
		}
		else
		{ 
		    components[i].middleX = components[i].sumX / components[i].area;
		    components[i].middleY = components[i].sumY / components[i].area;
		}
	}
	////std::cout << "cc2\n";
	
	
	//  Vector "components" is returned to the caller
}
