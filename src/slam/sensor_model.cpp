#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>
#include <cassert>
#include <cmath>
#include <iostream>

using namespace std;


SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
    z_hit = 0.8f;
    z_short = 0.05f;
    z_max = 0.1f;
    z_rand = 0.05f;

    lambda_short = 0.02f;
    sigma_hit = 0.05f;

    maxLaserDistance = 5.0f;
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    float q = 1.0f;

    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose, 1);

    int N = movingScan.size();
    float zStar;
    for(int i = 0; i < N ; i++){

        zStar = rayCasting(movingScan.at(i).origin.x, movingScan.at(i).origin.y, movingScan.at(i).theta, map);
        q *= z_hit * pHit(movingScan.at(i).range, zStar) + z_short * pShort(movingScan.at(i).range, zStar) + z_max * pMax(movingScan.at(i).range) + z_rand * pRand(movingScan.at(i).range);
    }
    q += 0.000000000001f;
    return q;
}

float SensorModel::rayCasting(float originX, float originY, float angle, const OccupancyGrid& map){

    float squareSide = map.metersPerCell();

    int N = floor(2 * maxLaserDistance / squareSide);

    float testPointX, testPointY;

    for(int i = 2; i < N; i++){
        testPointX = originX + i * squareSide * cos(angle) / 2;
        testPointY = originY + i * squareSide * sin(angle) / 2;
        Point<int> testPoint = global_position_to_grid_cell({testPointX, testPointY}, map);

        CellOdds odd = map.logOdds(testPoint.x, testPoint.y);

        if (odd > 0){
          return i * squareSide / 2;
        }
    }

    return maxLaserDistance;
}


float SensorModel::pHit(float distance, float zStar){
    if (distance <= maxLaserDistance) {
        return exp(- (distance - zStar)*(distance - zStar) / (2 * sigma_hit*sigma_hit)) / sqrt(2 * 3.1415926 * sigma_hit * sigma_hit);
    } else{
        return 0.0f;
    }
}

float SensorModel::pShort(float distance, float zStar){
    if(distance <= zStar){
      return lambda_short * exp(-lambda_short * distance);
    } else{
        return 0.0f;
    }
}

float SensorModel::pMax(float distance){
    if (distance == maxLaserDistance) {
        return 1.0f;
    } else{
        return 0.0f;
    }
}

float SensorModel::pRand(float distance){
    if(distance <= maxLaserDistance){
        return (1.0 / maxLaserDistance);
    } else{
        return 0.0f;
    }
}
