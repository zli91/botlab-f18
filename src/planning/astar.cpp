#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <bits/stdc++.h> 
#include <iostream>
#include <common/grid_utils.hpp>

using namespace std;

// std::ostream& operator<<(std::ostream& out, const pose_xyt_t& pose);
std::ostream& operator<<(std::ostream& out, const pose_xyt_t& pose)
{
    out << '(' << pose.x << ',' << pose.y << ',' << pose.theta << ')';
    return out;
};

// Creating a shortcut for int, int pair type 
typedef pair<int, int> Pair; 
  
// Creating a shortcut for pair<int, pair<int, int>> type 
typedef pair<double, pair<int, int>> pPair; 

struct cell 
{ 
    // Row and Column index of its parent 
    // Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1 
    int parent_i, parent_j; 
    // f = g + h 
    double f, g, h; 
}; 

void printPoint(Point<int> input)
{
	cout<<"a* printPair: ["<<input.x<<", "<<input.y<<"] "<<'\n';
}

Pair IndexSwap(int x, int y)
{ 
	Pair rowcol;
	rowcol.first = y;
	rowcol.second = x; 
	return rowcol; 
}

Pair Coor2Index(pose_xyt_t coor, float globalOx, float globalOy,float metersPerCell)
{ 
	int Indx = int(floor((coor.x - globalOx)/metersPerCell));
	int Indy = int(ceil((-coor.y - globalOy)/metersPerCell));
	//swapping
	Pair output = IndexSwap(Indx,Indy);
	return output;
}

pose_xyt_t Index2Coor(Pair Ind,  float globalOx, float globalOy,float metersPerCell)
{ 
	//swapping
	int Indx = Ind.second;
	int Indy = Ind.first;
	float Coorx = float(Indx) * metersPerCell + globalOx;
	float Coory = float(Indy) * metersPerCell + globalOy;
	pose_xyt_t output;
	output.x =  Coorx;
	output.y =  Coory;
	output.theta = 0.0;
	return output;
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
    // for(int y = 0; y < height_; ++y)
    // {
    //     for(int x = 0; x < width_; ++x)
    //     {
    //         // Unary plus forces output to be a a number rather than a character
    //         if(distances (x, y) >= float(60)){
    //         	cout << setw(4) << setprecision(3) << distances (x, y);
    //         }
    //         else if(distances (x, y) == float(0)){
    //         	cout << setw(4) << setprecision(3) << float(1);
    //         }
    //         else
    //         cout << setw(4) << setprecision(3) << min(distances (x, y),float(0.0));
    //     }
    //     cout << '\n';
    // }
    
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
    	// cout<<"Astar:DistCost if loop Enterred"<<'\n';
        distH += pow(params.maxDistanceWithCost - distances(i0,j0), 1.0 + params.distanceCostExponent);
    }
    return distH;
}

// A Utility Function to trace the path from the source 
// to destination 
robot_path_t tracePath(cell** cellDetails, pose_xyt_t start, pose_xyt_t goal, ObstacleDistanceGrid distances ,const SearchParams& params) 
{ 
	// float metersPerCell = distances.metersPerCell();
    // Point<float> globalOrigin = distances.originInGlobalFrame();
    // float globalOx = globalOrigin.x;
    // float globalOy = globalOrigin.y;
 //    Pair StartInd = Coor2Index(start,globalOx,globalOy,metersPerCell);
	// Pair DestInd = Coor2Index(goal,globalOx,globalOy,metersPerCell);
    Point<int> StartInd = global_position_to_grid_cell(Point<float>(start.x, start.y), distances);
    Point<int> DestInd = global_position_to_grid_cell(Point<float>(goal.x, goal.y), distances);
	robot_path_t path;
    // printf ("Astar:Tracing back\n"); 
    int row = DestInd.x; 
    int col = DestInd.y;

    float thetaWatcher = FLT_MAX;
    float currenttheta = 0.0;
  	// cout<< ""<<'\n';
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
    Pair input;

    while (!(cellDetails[row][col].parent_i == row 
             && cellDetails[row][col].parent_j == col )) 
    { 
    	// printf ("Astar Tracing: Enterred while loop\n");
        // Path.push (make_pair (row, col)); 
        input.first = row;
    	input.second = col;
    	
        // temp = Index2Coor(input, globalOx, globalOy, metersPerCell);
        tempcoor = grid_position_to_global_position(Point<float>(row,col),distances);
        int temp_row = cellDetails[row][col].parent_i; 
        int temp_col = cellDetails[row][col].parent_j; 

        int deli = temp_row - row;
        int delj = temp_col - col;

        

        if(distances(row,col) < params.maxDistanceWithCost * 1.0 ) 
        {   
            // cout<<"astar: tracepath: distance is actually:"<< distances(row,col) <<'\n';
            if(abs(delj) == 0 )
            {
                currenttheta = (float(deli) + 1.0)*M_PI/2.0;
            }
            else 
            {
                currenttheta = float((delj))* M_PI/2.0 + float((delj)*(deli)) * M_PI/4.0;
            }
            if(thetaWatcher != currenttheta )
            {
                temp.x = tempcoor.x;
                temp.y = tempcoor.y;
                temp.theta = 0.0;
                path.path.push_back(temp);
                // distances(row,col) = 76;
                thetaWatcher = currenttheta; 
                row = temp_row; 
                col = temp_col;
            }
            else
            {
                row = temp_row; 
                col = temp_col;
                continue;
            }

        }
        else
        {
            // distances(col,row) = 77;
            row = temp_row; 
            col = temp_col; 
            // continue;
        }
        
        // else
        // {   
        //     temp.x = tempcoor.x;
        //     temp.y = tempcoor.y;
        //     temp.theta = 0.0;
        //     path.path.push_back(temp);
        //     distances(row,col) = 76;
        //     // thetaWatcher = currenttheta; 
        //     row = temp_row; 
        //     col = temp_col;
        // }

        // cout<<"astar: tracepath: pushed once"<<'\n';
        

    } 
  
    // Path.push (make_pair (row, col)); 
    // while (!Path.empty()) 
    // { 
    //     pair<int,int> p = Path.top(); 
    //     Path.pop(); 
    //     printf("-> (%d,%d) ",p.first,p.second); 
    // } 
    path.path.push_back(start);
    // distances(row,col) = 66;
  	path.path_length = path.path.size();
  	cout<<"Astar:path_size is actually: "<<path.path.size()<<'\n';
  	reverse(path.path.begin(),path.path.end());
    cout<<"Astar: Printing the path: ";
    for(auto p = path.path.begin(); p != path.path.end()-1; ++p)
    {
        // if (&p != path.path.back()) 
        cout<< (*p) << "->";
        // else
        // cout<< &p << '\n';
    }
    cout<< path.path.back() <<'\n';
  	printSolMap(distances,StartInd,DestInd);
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
    // Point<float> globalOrigin = distances.originInGlobalFrame();
    // float globalOx = globalOrigin.x;
    // float globalOy = globalOrigin.y;
    bool closedList[width][height]; 
    memset(closedList, false, sizeof (closedList)); 
    // Pair StartInd = Coor2Index(start,globalOx,globalOy,gcoef);
    // Pair DestInd = Coor2Index(goal,globalOx,globalOy,gcoef);
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
	set<pPair> openList; 
    int insertion_counter = 0;
	openList.insert(make_pair (0.0, make_pair (i, j))); 
	bool foundDest = false; 
    int Row[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int Col[] = {0, 0, 1, -1, 1, -1, 1, -1};
    int curr_i, curr_j;

	while (!openList.empty()) 
	{
		// cout<<"Astar: Entered while loop"<<'\n';
        pPair p = *openList.begin(); 
  		
  		for(auto node : openList)
  		{
  			if(node.first <= p.first) 
  			{
  				p = node;
  			}
  		}

        // Remove this vertex from the open list 
        openList.erase(find(openList.begin(),openList.end(),p)); 
        // cout<<"a* current p(looping node):"<<'\n';
        // printPair(p.second);
        curr_i = p.second.first; 
        curr_j = p.second.second; 
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
	                return tracePath (cellDetails, start, goal, distances , params); 
	            } 
	            // If the successor is already on the closed 
	            // list or if it is blocked, then ignore it. 
	            // Else do the following 
	            else if (closedList[i][j] == false && distances(i,j) > params.minDistanceToObstacle ) 
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
	                    openList.insert( make_pair(fNew, make_pair(i, j))); 
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
        printSolMap(distances,StartInd,DestInd);
    	path.path.push_back(start); 
    	path.path_length = path.path.size();
    	return path;  
    }

}



