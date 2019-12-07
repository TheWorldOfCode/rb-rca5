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
#define BIG_WORLD 1
#define ENABLE_GLOBAL_POS 1

class FuzzyControl
{
public:
    FuzzyControl();

    void lidarCallback(ConstLaserScanStampedPtr & msg);
#if ENABLE_GLOBAL_POS == 1
    void poseCallbackNew(ConstPosesStampedPtr &_msg); // temp cheat method
#endif

    bool move(float & speed, float & dir);
    bool collect(float & speed, float & dir);
    void freeRoam(float & speed, float & dir);

    std::tuple<float, float> globMarble(const float mDir, const float mDist);
    void setMarble(float x, float y);
    void setGoal(const float x, const float y);
    std::tuple<float, float> calculateGoalDir(char c);
    std::tuple<float, float, float> getCoords();

    void drawRobotActualPath(float x, float y); /// Public because Dan might need it
    void saveRobotPathToFile(); /// Public because Dan might need it
    void drawRobotActualPath2(float x, float y, cv::Mat& map, cv::Vec3b color = {255, 0, 0});

    ~FuzzyControl();

private:
    std::vector<std::tuple<float, float>> lidar_data;
    bool flag ;
    fl::Engine * freeroamEngine;
    fl::InputVariable * obsDirRoam;
    fl::InputVariable * obsDistRoam;
    fl::OutputVariable * roamSteer;
    fl::OutputVariable * roamSpeed;

    fl::Engine * goalEngine;
    fl::InputVariable * obsDir;
    fl::InputVariable * obsDist;
    fl::InputVariable * goal;
    fl::InputVariable * goalDistance;
    fl::OutputVariable * steer;
    fl::OutputVariable * speed;

    fl::Engine * collectorEngine;
    fl::InputVariable * obsDirCol;
    fl::InputVariable * obsDistCol;
    fl::InputVariable * marbleDir;
    fl::InputVariable * marbleDist;
    fl::OutputVariable * collectSteer;
    fl::OutputVariable * collectSpeed;

    std::tuple<float, float, float> currentCoordinates;
    std::tuple< float, float> goalCoordinates;
    std::tuple<float, float, float> marbleCoordinates;
    boost::mutex  mutexFuzzy;

    cv::Mat map;
    cv::Mat mapCopy;

#if BIG_WORLD == 0
    float scaleFromPictureToModel =1/1.41735;
    float resizeFactor = 80;
    float combindedResizeFacotor=scaleFromPictureToModel*resizeFactor;
    float resizedWidth =20*combindedResizeFacotor*scaleFromPictureToModel; // width meaning x
    float resizedHeight =15*combindedResizeFacotor*scaleFromPictureToModel;
#else
    float scaleFromPictureToModel =1/1.41735;
    float resizeFactor = 15;
    float combindedResizeFacotor=scaleFromPictureToModel*resizeFactor;
    float resizedWidth =120*combindedResizeFacotor*scaleFromPictureToModel; // width meaning x
    float resizedHeight =80*combindedResizeFacotor*scaleFromPictureToModel;
#endif
};

