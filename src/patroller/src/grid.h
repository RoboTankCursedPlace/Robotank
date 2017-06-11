#ifndef GRID_H
#define GRID_H


#include "geometry_msgs/Point.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <climits>
#include <queue>

struct Node
{
	int row;    
	int column; 
	double h;
	double g;
	bool visited;
	Node* parent;
};

struct COMPARE
{
	bool operator()(Node n1, Node n2)
	{
		return (n1.h + n1.g > n2.h + n2.g);
	}
};


//  Returns the angle of line(P1, P2) and x-axis
double AngleBetweenPoints(double x1, double y1, double x2, double y2)
{
	double dy = y2-y1;
	double dx = x2-x1;
	return atan2(dy, dx);
}

//  Returns distance between P1 and P2 
double DistanceBetweenPoints(double x1, double y1, double x2, double y2)
{
	double dx = x2 - x1;
	double dy = y2 - y1;
	return sqrt( dx*dx + dy*dy );
}



class Grid
{
private:
    int width;
    int height;
    int** grid;
    bool** visited;
    double** f;
    
    std::vector<std::vector<struct Node> > grid_pf;
    
public:
    //  Construct a grid from a file
    Grid(const char* mapFileName);
	
	//  Conversion from grid[row][column] to point (x,y)
	geometry_msgs::Point Grid2Point(int row, int column);
	
	//  Conversion from point (x,y) to grid[row][column]
	bool Point2Grid(int &row, int &column, geometry_msgs::Point p);
	
	//  Returns distance between to grid locations
	double DistanceBetweenSquare(int row1, int column1, int row2, int column2);
	
	//  NOT USED
	bool WallAround(int row, int column);
	
	//  Prints map read from file
	void PrintMap();
	
	//  Finds a path (list of points) from grid[start] to grid[end]
	void AStar(int startRow, int startColumn, int goalRow, int goalColumn, std::list<geometry_msgs::Point> &path);
	
	//  Extends a path (list of points) by adding new points between each point in the path
	void ExtendPath(std::list<geometry_msgs::Point> &path, std::list<geometry_msgs::Point> &extendedPath);
	
	//  Prints a path (list of points)
	void PrintPath(std::list<geometry_msgs::Point> &path);
	
	//  Deallocate memory
	~Grid();
};



#endif
