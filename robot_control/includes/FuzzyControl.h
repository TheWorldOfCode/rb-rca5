#pragma once
//
// Created by lars on 10/14/19.
//
#include <vector>
#include <tuple>
#include <fl/Headers.h>
#include <gazebo/msgs/msgs.hh>

#define ENABLE_GLOBAL_POS 1

class FuzzyControl
{
public:
    FuzzyControl();
    void lidarCallback(ConstLaserScanStampedPtr & msg);
    void move(float & speed, float & dir);
    void setGoal(const float x, const float y);

#if ENABLE_GLOBAL_POS == 1
    void getCurrentPosition(ConstPosesStampedPtr &_msg); // temp cheat method
#endif

    ~FuzzyControl();
private:
    std::vector<std::tuple<float, float>> lidar_data;
    bool flag ;
    fl::Engine * engine;

    fl::InputVariable * obsDir;
    fl::InputVariable * obsDist;
    fl::InputVariable * goal;

    fl::OutputVariable * steer;
    fl::OutputVariable * speed;

    std::tuple<float, float, float> currentCoordinates;

    std::tuple< float, float> goalCoordinates;

};


