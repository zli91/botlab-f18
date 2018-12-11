#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>
#include <iostream>
#include <iomanip>
#include <fstream>
using namespace std;

ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
    
    resetGrid(map);
    int searchRange =max(width_,height_);
    bool disFound = false;
    // cout << "setDistance: enter for loop with a grid of size: ("<<width_<<", "<<height_<<") "<< std::endl;
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
    // cout << "setDistance: pre forloop"<< std::endl;
     for(int y = 0; y < height_; ++y)
        {
            // cout << "setDistance: enter 1st forloop"<< std::endl;
            for(int x = 0; x < width_; ++x)
            {
                // cout << "setDistance: enter 2nd forloop"<< std::endl;
                disFound = false;
                while(!disFound)
                 {
                     // cout << "setDistance: enter while"<< std::endl;
                     for (int i = 0; i <= searchRange ; ++ i)
                     {   
                         // cout << "setDistance: enter 1st while forloop"<< std::endl;
                         if(disFound){break;}
                         for (int j = 0; j <= i; ++ j)
                         {
                            // cout << "setDistance: enter 2nd while forloop"<< std::endl;
                            int permut8 [8] = {min(x+i,width_-1),max(x-i,0),min(x+j,width_-1),max(x-j,0),min(y+i,height_-1),max(y-i,0),min(y+j,height_-1),max(y-j,0) };

                            if (map.operator()(permut8[0],permut8[6])>=int8_t(0) || map.operator()(permut8[0],permut8[7])>=int8_t(0) || 
                                map.operator()(permut8[1],permut8[6])>=int8_t(0) || map.operator()(permut8[1],permut8[7])>=int8_t(0) || 
                                map.operator()(permut8[2],permut8[4])>=int8_t(0) || map.operator()(permut8[2],permut8[5])>=int8_t(0) || 
                                map.operator()(permut8[3],permut8[4])>=int8_t(0) || map.operator()(permut8[3],permut8[5])>=int8_t(0)) 
                            {
                                // operator()(x, y) = float(0) ;
                                operator()(x, y) = float(metersPerCell_ * sqrt(pow(i,2)+pow(j,2))) ;
                                disFound = true;
                                // std::cout << "setDistance: dist found"<< std::endl;
                                break;
                            }

                         }
                     }
                    if(!disFound)
                    {       
                    operator()(x, y) =99;
                    disFound = true;
                    }                                                
                 }    
            }
        }
        saveToFile("current_Obstacle_distance.txt");

        // cout<<"setDistance: after all loops "<<endl;
        // cout << left;
        // for(int y = 0; y < height_; ++y)
        // {
        //     for(int x = 0; x < width_; ++x)
        //     {
        //         // Unary plus forces output to be a a number rather than a character
        //         cout << setw(4) << setprecision(2) << operator()(x, y);
        //     }
        //     cout << '\n';
        // }
}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // std::cout << "resetGrid: enterred"<< std::endl;
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        std::cout << "resetGrid: return"<< std::endl;
        return;

    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
    // std::cout << "resetGrid: pre Save file"<< std::endl;
    map.saveToFile("current_Occu_Grid_Logodds.txt");

    // std::cout<<"Obstacle Dis grid size: "<< width_<<" ,"<<height_<<std::endl;
    // std::cout<<"Occu Grid LogOdds: "<<std::endl;
    // for(int i=0; i<int(cells_.size()); ++i)
    //     std::cout << cells_[i] << ' ';
    //     std::cout << std::endl;
}

bool ObstacleDistanceGrid::saveToFile(const std::string& filename) const
{
    std::ofstream out(filename);
    if(!out.is_open())
    {
        std::cerr << "ERROR: ObstacleDistanceGrid::saveToFile: Failed to save to " << filename << '\n';
        return false;
    }
    
    // Write header
    out << globalOrigin_.x << ' ' << globalOrigin_.y << ' ' << width_ << ' ' << height_ << ' ' << metersPerCell_ << '\n';
    
    // Write out each cell value
    for(int y = 0; y < height_; ++y)
    {
        for(int x = 0; x < width_; ++x)
        {
            // Unary plus forces output to be a a number rather than a character
             out << +int(ceil(operator()(x, y)));
        }
        out << '\n';
    }
    
    return out.good();
}