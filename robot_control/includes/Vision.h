#pragma once
//
// Created by lars on 10/14/19.
//
#include <vector>
#include <tuple>
#include <fl/Headers.h>
#include <gazebo/msgs/msgs.hh>
#include <opencv2/opencv.hpp>
#include <gazebo/gazebo_client.hh>




class Vision
{
public:
    Vision();

    void cameraCallbackRaw(ConstImageStampedPtr &msg);

    void cameraCallbackHough(ConstImageStampedPtr &msg);

    std::tuple<bool, float, float> getMarble();

    ~Vision();

private:
    std::tuple<float, float> calculateMarblePos(cv::Point, int);

    boost::mutex  mutexCamera;
    std::size_t width ;
    std::size_t height ;
    const char * data ;
    float marbleDir;
    float marbleDist;
    bool marbleFound;

};

