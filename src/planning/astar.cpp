#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <bits/stdc++.h> 
#include <iostream>
#include <common/grid_utils.hpp>
#include <common/timestamp.h>

using namespace std;

// std::ostream& operator<<(std::ostream& out, const pose_xyt_t& pose);
std::ostream& operator<<(std::ostream& out, const pose_xyt_t& pose)
{
    out << '(' << pose.x << ',' << pose.y << ',' << pose.theta << ')';
    return out;
};

struct cell 
{ 
    // Row and Column index of its parent 
    // Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1 
    int parent_i, parent_j; 
    // f = g + h 
    double f, g, h; 
}; 


bool BresenhamNoPass(int x1, int y1, int x2, int y2, ObstacleDistanceGrid distances, float thresh) 
{ 
// Bresenham's line algorithm
  const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
  if(steep)
  {
    std::swap(x1, y1);
    std::swap(x2, y2);
  }
 
  if(x1 > x2)
  {
    std::swap(x1, x2);
    std::swap(y1, y2);
  }
 
  const float dx = x2 - x1;
  const float dy = fabs(y2 - y1);
 
  float error = dx / 2.0f;
  const int ystep = (y1 < y2) ? 1 : -1;
  int y = (int)y1;
 
  const int maxX = (int)x2;
 
  for(int x=(int)x1; x<maxX; x++)
  {
    if(distances(y,x) <= thresh)
    {
       return true;
    }
    error -= dy;
    if(error < 0)
    {
        y += ystep;
        error += dx;
    }
    
        if(steep && (distances(y,x) <= thresh))
    {
        // cout << "(" << y << "," << x << ")\n";
        return true;

    }
    else if(distances(y,x) <= thresh )
    {
       return true;
    }
  }
  return false;
} 



void printPoint(Point<int> input)
{
	cout<<"a* printPair: ["<<input.x<<", "<<input.y<<"] "<<'\n';
}

bool isValid(int row, int col, int width, int height) 
{ 
    // Returns true if row number and column number 
    // is in range 
    return (row >= 0) && (row < width) && 
           (col >= 0) && (col < height); 
} 

bool isDestination(int row, int col, Point<int> dest) 
{ 
    if (row == dest.x && col == dest.y) 
        return (true); 
    else
        return (false); 
} 

void printSolMap(ObstacleDistanceGrid distances, Point<int> start, Point<int> goal)
{

    cout<<"setDistance: after all loops "<<endl;
    cout << left;
    int width_ = distances.widthInCells();
    int height_ = distances.heightInCells(); 
    for(int y = 0; y < height_; ++y)
    {
        for(int x = 0; x < width_; ++x)
        {
            // Unary plus forces output to be a a number rather than a character
            if(distances (x, y) == float(77)){
                distances (x, y) = 7;
            }
            else if(distances (x, y) == float(76)){
                distances (x, y) = 9;
            }
            else if(distances (x, y) == float(0)){
                distances (x, y) = 0;
            }
            else {
                distances (x, y) = 1;
            }
            
        }
        // cout << '\n';
    }
    distances (start.x, start.y) = float(6);
    distances (goal.x, goal.y) = float(8);
    distances.saveToFile("current_SOL_map.txt");
}
double distCost(int i0, int j0, Point<int> dest, const ObstacleDistanceGrid& distances,const SearchParams& params)
{
	int i1 = dest.x;
	int j1 = dest.y;

    double distH = double(distances.metersPerCell()) * sqrt(pow((i0-i1),2) + pow((j0-j1),2));

    // Pair swap = IndexSwap(i0,j0);
	// i0 = swap.first;
	// j0 = swap.second;
    if (distances(i0,j0)>params.minDistanceToObstacle && distances(i0,j0)<params.maxDistanceWithCost)
    {
        double Obs = abs(pow((params.maxDistanceWithCost - distances(i0,j0)), params.distanceCostExponent));
    	// cout<<"Astar:ObsCost:"<<Obs<<'\n';
        // cout<<"Astar:EucCost:"<<distH<<'\n';
        distH += Obs;

    }
    return distH;
}

// A Utility Function to trace the path from the source 
// to destination 
robot_path_t tracePath(cell** cellDetails, pose_xyt_t start, pose_xyt_t goal, ObstacleDistanceGrid distances ,const SearchParams& params) 
{ 
    int64_t starttime = utime_now();
    Point<int> StartInd = global_position_to_grid_cell(Point<float>(start.x, start.y), distances);
    Point<int> DestInd = global_position_to_grid_cell(Point<float>(goal.x, goal.y), distances);
	robot_path_t path;
    // printf ("Astar:Tracing back\n"); 
    int row = DestInd.x; 
    int col = DestInd.y;
    int Bx1 = StartInd.x;
    int By1 = StartInd.y;

    float thetaWatcher = FLT_MAX;
    float currenttheta = 0.0;
  	// cout<< "stops1"<<'\n';
  	// cout<<"Astar Tracing: checking DestInd" <<'\n';
  	// printPair(DestInd);
    // stack <Pair> Path
  	path.utime = start.utime;
  	path.path.push_back(goal);

  	// distances(col,row) = 88;

  	pose_xyt_t temp;
  	Point<double> tempcoor;
    row = cellDetails[row][col].parent_i; 
    col = cellDetails[row][col].parent_j; 
    // cout<<"Astar Tracing: checking DestInd's parent" <<'\n';
  	// cout<<"a* printPair: ["<<row<<", "<<col<<"] "<<'\n';
    // cout<< "stops2"<<'\n';
    while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col )) 
    { 
    	// printf ("Astar Tracing: Enterred while loop\n");
        tempcoor = grid_position_to_global_position(Point<float>(row,col),distances);
        int temp_row = cellDetails[row][col].parent_i; 
        int temp_col = cellDetails[row][col].parent_j; 

        int deli = temp_row - row;
        int delj = temp_col - col;  
         

        // if((distances(row,col) < params.maxDistanceWithCost )) 
        // {   
            // if ( distances(row,col) < params.minDistanceToObstacle * 1.05  )
            // {
            //     temp.x = tempcoor.x;
            //     temp.y = tempcoor.y;
            //     temp.theta = 0.0;
            //     path.path.push_back(temp);
            //     // distances(row,col) = 76;
            //     thetaWatcher = currenttheta; 
            //     row = temp_row; 
            //     col = temp_col;
            //     continue;
            // }
            // else
            // {
                                // cout<<"astar: tracepath: distance is actually:"<< distances(row,col) <<'\n';
                if(abs(delj) == 0 )
                {
                    currenttheta = (float(deli) + 1.0)*M_PI/2.0;
                    // cout<<"astar: currenttheta:"<< currenttheta<<'\n';
                }
                else 
                {
                    currenttheta = float((delj))* M_PI/2.0 + float((delj)*(deli)) * M_PI/4.0;
                    // cout<<"astar: currenttheta:"<< currenttheta<<'\n';
                }
                if(thetaWatcher != currenttheta &&BresenhamNoPass(Bx1,By1,temp_row,temp_col,distances,1.5 * params.minDistanceToObstacle))
                {
                    // cout<<"astar: telling theta:"<<'\n';
                    
                    temp.x = tempcoor.x;
                    temp.y = tempcoor.y;
                    temp.theta = 0.0;
                    path.path.push_back(temp);
                    // distances(row,col) = 76;
                    thetaWatcher = currenttheta; 
                    row = temp_row; 
                    col = temp_col;
                    // cout<<"astar: done telling theta:"<<'\n';                    
                }
                else
                {
                    // cout<<"astar: dont push:"<<'\n';
                    row = temp_row; 
                    col = temp_col;
                    continue;
                }
            // }
        // }
        // else
        // {
        //     // distances(col,row) = 77;
        //     row = temp_row; 
        //     col = temp_col; 
        //     // continue;
        // }
        // cout <<"printing current parent:[" << cellDetails[row][col].parent_i<<", "<<cellDetails[row][col].parent_j<<"]"<<'\n';
    } 
  

    path.path.push_back(start);
    // distances(row,col) = 66;
  	path.path_length = path.path.size();
  	// cout<<"Astar:path_size is actually: "<<path.path.size()<<'\n';
  	reverse(path.path.begin(),path.path.end());
    // cout<<"Astar: Printing the path: ";
    for(auto p = path.path.begin(); p != path.path.end()-1; ++p)
    {
        // if (&p != path.path.back()) 
        cout<< (*p) << "->";
        // else
        // cout<< &p << '\n';
    }
    cout<< path.path.back() <<'\n';
    int64_t endtime = utime_now();
    // cout<< "time to pass trace map:" << endtime - starttime <<'\n';
    return path; 
} 


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{	
	// std::cout << "a* test: start's theta: " << start.theta <<'\n';
	// cout<<"Astar:Enterred a*"<<'\n';
    ////////////////// TODO: Implement your A* search here //////////////////////////  
    
    int width = distances.widthInCells();
    int height = distances.heightInCells();
    float gcoef = distances.metersPerCell();
    bool closedList[width][height]; 
    memset(closedList, false, sizeof (closedList)); 
    Point<int> StartInd = global_position_to_grid_cell(Point<float>(start.x, start.y), distances);
    Point<int> DestInd = global_position_to_grid_cell(Point<float>(goal.x, goal.y), distances);
    cout<<"a*: start and gold index:"<<'\n';
    printPoint(StartInd);
    printPoint(DestInd);
    // cell cellDetails[width][height]; 
    cell **cellDetails;
	cellDetails = new cell*[width]; // dynamic array (size 10) of pointers to int

	for (int i = 0; i < width; ++i) 
	{
  	cellDetails[i] = new cell[height];
  // each i-th pointer is now pointing to dynamic array (size 10) of actual int values
	}
	// cout<<"Initiated cellDetails"<<'\n';

    int i, j; 
  
    for (i=0; i<width; i++) 
    { 
        for (j=0; j<height; j++) 
        { 
            cellDetails[i][j].f = FLT_MAX; 
            cellDetails[i][j].g = FLT_MAX; 
            cellDetails[i][j].h = FLT_MAX; 
            cellDetails[i][j].parent_i = -1; 
            cellDetails[i][j].parent_j = -1; 
        } 
    } 

    i = StartInd.x, j = StartInd.y; 
    cellDetails[i][j].f = 0.0; 
    cellDetails[i][j].g = 0.0; 
    cellDetails[i][j].h = 0.0; 
    cellDetails[i][j].parent_i = i; 
    cellDetails[i][j].parent_j = j; 
    // cout<<"Astar: Ready Prep cellDetails"<<'\n';
// Put the starting cell on the open list and set its 
// 'f' as 0 
	// set<pPair> openList; 
    std::vector<Point<int>> openListv; 
    int insertion_counter = 0;
	// openList.insert(make_pair (0.0, make_pair (i, j))); 
    openListv.push_back(Point<int>(i,j));
	bool foundDest = false; 
    int Row[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int Col[] = {0, 0, 1, -1, 1, -1, 1, -1};
    int curr_i, curr_j;
    int64_t starttime = utime_now();
    int counter = 0;
	while (!openListv.empty()) 
	{
		// cout<<"Astar: Entered while loop"<<'\n';

        Point<int> pt = *openListv.begin();

        for (auto node : openListv)
        {
            if(cellDetails[node.x][node.y].h < cellDetails[pt.x][pt.y].h )
            {
                pt = node;
            }
        }

        // Remove this vertex from the open list 
        openListv.erase(find(openListv.begin(),openListv.end(),pt)); 
        // cout<<"a* current p(looping node):"<<'\n';
        curr_i = pt.x; 
        curr_j = pt.y; 

        closedList[curr_i][curr_j] = true; 

		/* 
        Generating all the 8 successor of this cell 
  
            N.W   N   N.E 
              \   |   / 
               \  |  / 
            W----Cell----E 
                 / | \ 
               /   |  \ 
            S.W    S   S.E 
  
        Cell-->Popped Cell (i, j) 
        N -->  North       (i-1, j) 
        S -->  South       (i+1, j) 
        E -->  East        (i, j+1) 
        W -->  West           (i, j-1) 
        N.E--> North-East  (i-1, j+1) 
        N.W--> North-West  (i-1, j-1) 
        S.E--> South-East  (i+1, j+1) 
        S.W--> South-West  (i+1, j-1)*/
        float diagcost = 1.2;
        float Movecost[] = {1.0, 1.0, 1.0, 1.0, diagcost, diagcost, diagcost, diagcost};
        // To store the 'g', 'h' and 'f' of the 8 successors 
        double gNew, hNew, fNew;

        for (int iter = 0; iter < 8 ; iter++ )
        {	
            counter ++;
        	// cout<<"Astar: Entered 8 iteration loops and dist(j,i) = "<< distances(j,i) <<'\n';
        	i = curr_i + Row[iter];
        	j = curr_j + Col[iter];
        	if (isValid(i, j, width, height) == true) 
	        { 
                // cout<<"Astar: Entered 8 iteration loops: and cell is valid"<<'\n';
	            // If the destination cell is the same as the 
	            // current successor 
	            if (isDestination(i, j, DestInd) == true) 
	            { 
	                // Set the Parent of the destination cell 
	                cellDetails[i][j].parent_i = curr_i; 
	                cellDetails[i][j].parent_j = curr_j; 
	                printf ("Astar: The destination cell is found\n"); 
	                foundDest = true; 
                    // delete cellDetails;
                    int64_t endtime = utime_now();
                    cout << "time to pass astar:" << endtime - starttime << "iterations:" << counter <<'\n';
	                return tracePath (cellDetails, start, goal, distances , params); 
	            } 
	            // If the successor is already on the closed 
	            // list or if it is blocked, then ignore it. 
	            // Else do the following 
	            else if (closedList[i][j] == false && distances(i,j) > params.minDistanceToObstacle) 
	            { 
	            	
	                gNew = cellDetails[curr_i][curr_j].g + Movecost[iter] * gcoef; 
	                hNew = distCost (i, j, DestInd, distances, params); 
	                fNew = gNew + hNew; 
	  				// cout<<"A*: Entered main pathing loop and the current gcost is: "<<gNew<<'\n';
	  				// cout<<"A*: Entered main pathing loop and the current hcost is: "<<hNew<<'\n';
	                // If it isn’t on the open list, add it to 
	                // the open list. Make the current square 
	                // the parent of this square. Record the 
	                // f, g, and h costs of the square cell 
	                //                OR 
	                // If it is on the open list already, check 
	                // to see if this path to that square is better, 
	                // using 'f' cost as the measure. 
	                if (cellDetails[i][j].f == FLT_MAX || cellDetails[i][j].f > fNew) 
	                { 
                        openListv.push_back(Point<int>(i,j));
	                    insertion_counter++;
	                    // Update the details of this cell 
	                    cellDetails[i][j].f = fNew; 
	                    cellDetails[i][j].g = gNew; 
	                    cellDetails[i][j].h = hNew; 
	                    cellDetails[i][j].parent_i = curr_i; 
	                    cellDetails[i][j].parent_j = curr_j; 
	                } 
	            } 
	        }
        }

	}

    if (foundDest == false) 
    {
        cout<<"Astar: Failed to find the Destination Cell with insertion: "<< insertion_counter << '\n'; 
    	robot_path_t path;
    	path.utime = start.utime;
        // printSolMap(distances,StartInd,DestInd);
    	path.path.push_back(start); 
    	path.path_length = path.path.size();
    	return path;  
    }

}



