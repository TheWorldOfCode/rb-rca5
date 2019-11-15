#pragma once
//
// Created by lars on 10/14/19.
//
#include <vector>
#include <tuple>
#include <fl/Headers.h>
#include <gazebo/msgs/msgs.hh>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <gazebo/gazebo_client.hh>

#define ENABLE_GLOBAL_POS 1

class FuzzyControl
{
public:
    FuzzyControl();
    void lidarCallback(ConstLaserScanStampedPtr & msg);
    void move(float & speed, float & dir);
    void setGoal(const float x, const float y);

    bool collect(float & speed, float & dir);
    std::tuple<float, float> globMarble(const float mDir, const float mDist);
    void setMarble(float x, float y);

    void drawRobotActualPath(float x, float y); /// Public because Dan might need it
    void saveRobotPathToFile(); /// Public because Dan might need it

#if ENABLE_GLOBAL_POS == 1
    void poseCallbackNew(ConstPosesStampedPtr &_msg); // temp cheat method
#endif

    float calculateGoalDir(char c);

    ~FuzzyControl();
private:
    std::vector<std::tuple<float, float>> lidar_data;
    bool flag ;
    fl::Engine * roamEngine;
    fl::InputVariable * obsDir;
    fl::InputVariable * obsDist;
    fl::InputVariable * goal;
    fl::OutputVariable * steer;
    fl::OutputVariable * speed;
    std::tuple<float, float, float> currentCoordinates;
    std::tuple< float, float> goalCoordinates;
    boost::mutex  mutexFuzzy;

    fl::Engine * collectorEngine;
    fl::InputVariable * obsDirCol;
    fl::InputVariable * obsDistCol;
    fl::InputVariable * marbleDir;
    fl::InputVariable * marbleDist;
    fl::OutputVariable * collectSteer;
    fl::OutputVariable * collectSpeed;
    std::tuple<float, float, float> marbleCoordinates;

    cv::Mat map;
    cv::Mat mapCopy;

    float scaleFromPictureToModel =1/1.41735;
    float resizeFactor = 80;
    float combindedResizeFacotor=scaleFromPictureToModel*resizeFactor;
    float resizedWidth =20*combindedResizeFacotor*scaleFromPictureToModel; // width meaning x
    float resizedHeight =15*combindedResizeFacotor*scaleFromPictureToModel;

};

