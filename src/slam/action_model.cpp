#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>
#include <cmath>
#include <iostream>

using namespace std;


ActionModel::ActionModel(void)
: firstCall_(true)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    alpha[0] = alpha[1] = 0.1;
    alpha[2] = alpha[3] = 0.01;
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////

    if(firstCall_){
      currentOdometry = odometry;
      firstCall_ = false;
      return false;
    }

    previousOdometry = currentOdometry;

    currentOdometry = odometry;

    float dRot1 = atan2(currentOdometry.x - previousOdometry.x, currentOdometry.y - previousOdometry.y) - previousOdometry.theta;
    float dTrans = sqrt((currentOdometry.x - previousOdometry.x)*(currentOdometry.x - previousOdometry.x) + (currentOdometry.y - previousOdometry.y)*(currentOdometry.y - previousOdometry.y));
    float dRot2 = currentOdometry.theta - previousOdometry.theta - dRot1;

    if(dRot1*dRot1 + dTrans*dTrans + dRot2*dRot2 < 0.0003){
      return false;
    }

    std::random_device mch;
    std::default_random_engine generator(mch());

    std::normal_distribution<double> distributionRot1(0.0, alpha[0]*dRot1*dRot1 + alpha[1]*dTrans*dTrans);
    std::normal_distribution<double> distributionTrans(0.0, alpha[2]*dTrans*dTrans + alpha[3]*dRot1*dRot1 + alpha[3]*dRot2*dRot2);
    std::normal_distribution<double> distributionRot2(0.0, alpha[0]*dRot2*dRot2 + alpha[1]*dTrans*dTrans);

    hatdRot1 = dRot1 - distributionRot1(generator);
    hatdTrans = dTrans - distributionTrans(generator);
    hatdRot2 = dRot2 - distributionRot2(generator);

    return true;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

    particle_t newSample;

    newSample.parent_pose = sample.pose;

    float delta_x = hatdTrans*cos(sample.pose.theta + hatdRot1);
    float delta_y = hatdTrans*sin(sample.pose.theta + hatdRot1);
    float delta_theta = hatdRot1 + hatdRot2;

    newSample.pose.x = sample.pose.x + delta_x;
    newSample.pose.y = sample.pose.y + delta_y;
    newSample.pose.theta = sample.pose.theta + delta_theta;

    return newSample;

}
