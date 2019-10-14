#pragma once
//
// Created by lars on 10/14/19.
//
#include <vector>
#include <tuple>
#include <fl/Headers.h>
#include <gazebo/msgs/msgs.hh>

class FuzzyControl
{
public:
    FuzzyControl();
    void lidarCallback(ConstLaserScanStampedPtr & msg);
    void move(float & speed, float & dir);
    ~FuzzyControl();
private:
    std::vector<std::tuple<float, float>> lidar_data;
    bool flag ;
    fl::Engine * engine;

    fl::InputVariable * obstacle;
    fl::InputVariable * goal;

    fl::OutputVariable * steer;
};


