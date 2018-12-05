#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <lcmtypes/lidar_t.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>
#include <iostream>
#include <stdio.h>
#include <vector>

using namespace std;

// Constructor for the Mapping class

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, firstUpdate_(1)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////

  if(firstUpdate_){
    currentPose = pose;
    firstUpdate_ = 0;
    return;
  }

  lastPose = currentPose;

  currentPose = pose;

  MovingLaserScan movingScan(scan, lastPose, pose, 1);

  int N = movingScan.size();


  for(int i = 0; i < N ; i++){
    double botX = movingScan.at(i).origin.x;
    double botY = movingScan.at(i).origin.y;
    Point<int> botCellXY = global_position_to_grid_cell({botX, botY}, map);

    if (movingScan.at(i).range < kMaxLaserDistance_){
      double hitX = movingScan.at(i).origin.x + movingScan.at(i).range*cos(movingScan.at(i).theta);
      double hitY = movingScan.at(i).origin.y + movingScan.at(i).range*sin(movingScan.at(i).theta);
      Point<int> hitCellXY = global_position_to_grid_cell({hitX, hitY}, map);

      bresenhamLineUpdate(botCellXY, hitCellXY, map);

    }
  }

  if(map.saveToFile("../data/myMap.map")){
      std::cout << "Map updated!" << '\n';
  }


  return;
}

/**
* Converts a (x, y) point to the corresponding grid map cell.
* We may not use it because there is a different function in grid_utils.hpp
* that does exactly the same thing;
**/
Point<int> Mapping::xy2Cell(const float x, const float y, OccupancyGrid& map){
  float globalPositionX = x;
  float globalPositionY = y;

  int gridWidth = map.widthInCells();
  int gridHeight = map.heightInCells();
  float cellSide = map.metersPerCell();

  int gridPositionX = int(floor(globalPositionX / cellSide) + gridWidth/2);
  int gridPositionY = int(floor(globalPositionY / cellSide) + gridHeight/2);

  return {gridPositionX, gridPositionY};
}


/**
* Updates logOdds of the cells in between the cells firstCell and secondCell
* using Bresenham algorithm
**/
void Mapping::bresenhamLineUpdate(Point<int> firstCell, Point<int> secondCell, OccupancyGrid& map)
{
  if(map.isCellInGrid(firstCell.x, firstCell.y) && map.isCellInGrid(secondCell.x, secondCell.y)){

    // Setup initial conditions
    int x0 = firstCell.x;
    int y0 = firstCell.y;
    int x1 = secondCell.x;
    int y1 = secondCell.y;

    int dx = x1 - x0;
    int dy = y1 - y0;

    // Determine if the line is steep. If so, rotate;
    int isSteep = std::abs(dy) > std::abs(dx) ? 1 : 0;

    if (isSteep){
      int tmpx = x0;
      int tmpy = y0;
      x0 = tmpy;
      y0 = tmpx;
      tmpx = x1;
      tmpy = y1;
      x1 = tmpy;
      y1 = tmpx;
    }

    int swap = 0;
    if (x0 > x1){
      int tmpx0 = x0;
      int tmpy0 = y0;
      x0 = x1;
      y0 = y1;
      x1 = tmpx0;
      y1 = tmpy0;

      swap = 1;
    }

    // Recalculate differentials
    dx = x1 - x0;
    dy = y1 - y0;

    // Calculate error
    int error = dx / 2.0;
    int sy = y0 < y1 ? 1 : -1;

    if (!swap){
      int y = y0;
      for(int x = x0; x < x1; x++){
        int coord[2];
        if(isSteep){
          coord[0] = y;
          coord[1] = x;
        } else{
          coord[0] = x;
          coord[1] = y;
        }


        int coordOdd = map.logOdds(coord[0], coord[1]);
        map.setLogOdds(coord[0], coord[1], clampCellLogOdds(coordOdd - kMissOdds_));

        error -= std::abs(dy);
        if (error < 0){
          y += sy;
          error += dx;
        }
      }
      if(!isSteep){
        int coordOdd = map.logOdds(x1, y1);
        map.setLogOdds(x1, y1, clampCellLogOdds(coordOdd + kHitOdds_));
      } else{
          int coordOdd = map.logOdds(y1, x1);
          map.setLogOdds(y1, x1, clampCellLogOdds(coordOdd + kHitOdds_));
      }
    }
    else{
      int y = y1;
      for(int x = x1; x > x0; x--){
        int coord[2];
        if(isSteep){
          coord[0] = y;
          coord[1] = x;
        } else{
          coord[0] = x;
          coord[1] = y;
        }


        int coordOdd = map.logOdds(coord[0], coord[1]);
        map.setLogOdds(coord[0], coord[1], clampCellLogOdds(coordOdd - kMissOdds_));

        error -= std::abs(dy);
        if (error < 0){
          y -= sy;
          error += dx;
        }
      }
      if(!isSteep){
        int coordOdd = map.logOdds(x0, y0);
        map.setLogOdds(x0, y0, clampCellLogOdds(coordOdd + kHitOdds_));
      } else{
        int coordOdd = map.logOdds(y0, x0);
        map.setLogOdds(y0, x0, clampCellLogOdds(coordOdd + kHitOdds_));
      }
    }
    return;
  }

  else{
    std::cout << "Error updating map, cells are not in the grid.\n";
    return;
  }
}

int Mapping::clampCellLogOdds(int logOdd){
  if(logOdd > 127){ return 127; }
  else if(logOdd < -128) { return -128; }
  else { return logOdd; }
}

// void Mapping::setLastPose(const float & poseX, const float & poseY, const float & poseT){
//
//     lastPose.x = poseX;
//     lastPose.y = poseY;
//     lastPose.theta = poseT;
//     return;
// }
