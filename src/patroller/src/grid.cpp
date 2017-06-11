#include "grid.h"


Grid::Grid(const char* mapFileName)
{
	std::ifstream infile(mapFileName);
	
	//  Read map dimensions
	infile >> height >> width;
		
		
	//  Create 2d array
	grid = new int*[height];
	visited = new bool*[height];
	f = new double*[height];
	for (int i = 0; i < height; ++i)
	{
		grid[i] = new int[width];
		visited[i] = new bool[width];
		f[i] = new double[width];
	}
	
	grid_pf.resize(height, std::vector<struct Node>(width));
		
		
	//  Read grid from txt file
	for (int i = 0; i < height; ++i)
	{
		for (int j = 0; j < width; ++j)
		{
			infile >> grid[i][j];
		}
	}
		
		
	infile.close();
}
	
		
geometry_msgs::Point Grid::Grid2Point(int row, int column)
{
	geometry_msgs::Point result;
	result.x = 10.5 - row;
	result.y = 15.5 - column;
	result.z = 0;
		
	return result;
}
	
	
bool Grid::Point2Grid(int &row, int &column, geometry_msgs::Point p)
{
	bool result = true;
		
	float x = 10.5 - p.x + 0.5; // + 0.5;
	float y = 15.5 - p.y + 0.5; // + 0.5;
		
		
	//  Check if top left is empty
	row = x;
	column = y;
	//std::cout << row << ", " << column << std::endl;	
	if (!grid[row][column])
	{
		return true;
	}
	
	
	//  Check if top right is empty
	row = x;
	column = y + 1;
	if (!grid[row][column])
	{
		return true;
	}
		
		
	//  Check if bot left is empty
	row = x + 1;
	column = y;
	if (!grid[row][column])
	{
		return true;
	}
		

	//  Check if bot right is empty
	row = x + 1;
	column = y + 1;
	if (!grid[row][column])
	{
		return true;
	}
		
		
	//  Unable to find an empty neighbor square
	row = -1;
	column = - 1;
	return false;
}

double Grid::DistanceBetweenSquare(int row1, int column1, int row2, int column2)
{
	geometry_msgs::Point p1 = Grid2Point(row1, column1);
	geometry_msgs::Point p2 = Grid2Point(row2, column2);
	
	double dx = p1.x - p2.x;
	double dy = p1.y - p2.y;
	
	double dist = sqrt(dx*dx + dy*dy);
	
	return dist;
}
	    
bool Grid::WallAround(int row, int column)
{
	return false;
}
	
void Grid::PrintMap()
{
	std::cout << "--- Printing Grid ---" << std::endl;
	for (int i = 0; i < height; ++i)
	{
		for (int j = 0; j < width; ++j)
		{
			std::cout << grid[i][j] << " ";
		}
		std::cout << std::endl;
	}
}


//  Directions for 8 neighbor
int dirRow[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
int dirColumn[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

void Grid::AStar(int startRow, int startColumn, int goalRow, int goalColumn, std::list<geometry_msgs::Point> &path)
{
	for (int i = 0; i < height; ++i)
	{
		for (int j = 0; j < width; ++j)
		{
			grid_pf[i][j].row = i;
			grid_pf[i][j].column = j;
			grid_pf[i][j].h = DistanceBetweenSquare(i, j, goalRow, goalColumn);
			grid_pf[i][j].g = 100000;
			grid_pf[i][j].visited = false;
			//f[i][j] = 
		}
	}
	
	

	std::cout << "Starting A*" << std::endl;
	
	//  Starting point
	//grid_pf[startRow][startColumn].h = DistanceBetweenSquare(startRow, startColumn, goalRow, goalColumn);
	grid_pf[startRow][startColumn].g = 0;
	grid_pf[startRow][startColumn].parent = NULL;
	std::priority_queue<struct Node, std::vector<struct Node>, COMPARE> Q;
	Q.push(grid_pf[startRow][startColumn]);
	//visited[startRow][startColumn] = true;
	
	
	while (!Q.empty())
	{
		struct Node current = Q.top();
		Q.pop();
		
		grid_pf[current.row][current.column].visited = true;
		
		std::cout << std::endl;
		
		std::cout << "Size of queue:" << Q.size() << std::endl;
		std::cout << "Expanded node:" << "[" << current.row << "][" << current.column << "]" << std::endl;
		
		std::cout << "h:" << grid_pf[current.row][current.column].h << std::endl;
		std::cout << "g:" << grid_pf[current.row][current.column].g << std::endl;
		std::cout << "f:" << grid_pf[current.row][current.column].h + grid_pf[current.row][current.column].g << std::endl;
		
		//  Check if we reached the goal
		if (current.row == goalRow && current.column == goalColumn)
		{
			std::cout << "A* FINISHED !!!" << std::endl;
			//  traceback from goal, add points
			do
			{
				geometry_msgs::Point p = Grid2Point(current.row, current.column);
				path.push_front(p);
				current = *current.parent;
			} while(! (current.row == startRow && current.column == startColumn) );
			
			return;
		}
		
	
	    //  Check all neighbors
		for (int dir = 0; dir < 8; ++dir)
		{
			int neighborRow = current.row + dirRow[dir];
			int neighborColumn = current.column + dirColumn[dir];
			
			//  Check if square is valid
			if (neighborRow >= 0 && neighborRow < height && neighborColumn >= 0 && neighborColumn < width)
			{
				//  Check if walkable
				if (!grid[neighborRow][neighborColumn])
				{
					//  Check if not visited
					if (!visited[neighborRow][neighborColumn])
					{
						std::cout << "checking square:" << "[" << neighborRow << "][" << neighborColumn << "]" << std::endl;
						double newg = grid_pf[current.row][current.column].g + DistanceBetweenSquare(current.row, current.column, neighborRow, neighborColumn);
						
						
						
						std::cout << "newg:" << newg << std::endl;
						std::cout << "oldg:" << grid_pf[neighborRow][neighborColumn].g << std::endl;
						
						
						if (newg < grid_pf[neighborRow][neighborColumn].g)
						{
							std::cout << "adding to Q" << std::endl;
							grid_pf[neighborRow][neighborColumn].g = newg;
							grid_pf[neighborRow][neighborColumn].parent = &(grid_pf[current.row][current.column]);
							Q.push(grid_pf[neighborRow][neighborColumn]);
						}
						
						//grid_pf[neighborRow][neighborColumn].h = DistanceBetweenPoints(current.row, current.column, goalRow, goalColumn);
						//grid_pf[
					}
				}
			}
		}
		
		
	}
	
	
	//  Unable to find a path, return empty list
	path.clear();			
}

void Grid::ExtendPath(std::list<geometry_msgs::Point> &path, std::list<geometry_msgs::Point> &extendedPath)
{
	const int extra = 4;
	
	for (std::list<geometry_msgs::Point>::iterator current = path.begin(); current != path.end(); ++current)
	{
		std::list<geometry_msgs::Point>::iterator next = current;
		++next;
		
		if (next == path.end())
		{
			extendedPath.push_back(*current);
			break;
		}
		
		
		double distX = (next->x - current->x) / (extra + 1);
		double distY = (next->y - current->y) / (extra + 1);
		
		extendedPath.push_back(*current);
		
		for (int i = 0; i < extra; ++i)
		{
			geometry_msgs::Point p;
			p.x = current->x + distX * (i+1);
			p.y = current->y + distY * (i+1);
			extendedPath.push_back(p);
		}
		
	}
}

void Grid::PrintPath(std::list<geometry_msgs::Point> &path)
{
	for (std::list<geometry_msgs::Point>::iterator it = path.begin(); it != path.end(); ++it)
	{
		std::cout << "Point "  << ":" << "(" << it->x << "," << it->y << ")" << std::endl;
		
		double x = it->x;
		double y = it->y;
		
		int rr;
		int cc;
		Point2Grid(rr, cc, *it);
		
		std::cout << "Grid " << ":" << "[" << rr << "][" << cc << "]" << std::endl;
	}
}
	
	
	
Grid::~Grid()
{
	for (int i = 0; i < height; ++i)
	{
		delete [] grid[i];
	}
		
	delete [] grid;
}
