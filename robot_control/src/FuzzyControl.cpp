//
// Created by lars on 10/14/19.
//
#define FUZZY_DEBUG 0
#define DRAW_PATH 0
#define FREE_ROAM 0
#define OVERRIDE_CV_MARBLE 0

#include <string>
#include <cmath>        // std::atan2

#if FUZZY_DEBUG == 1
#include <iostram>
#endif

#include "../includes/FuzzyControl.h"

using namespace fl;

FuzzyControl::FuzzyControl()
{
    flag = false;
    //################## Goal Engine ##################################

    goalEngine = FllImporter().fromFile(
            "../fuzzy_control/GoalAndObstacleAvoidance.fll");// bemærk et niveau op, kunne også have flyttet .fll

    std::string status;
    if (not goalEngine->isReady(&status))
        throw Exception("[goalEngine error] goalEngine is not ready:n" + status, FL_AT);

    obsDir = goalEngine->getInputVariable("obsDir");
    obsDist = goalEngine->getInputVariable("obsDist");
    goal = goalEngine->getInputVariable("goal");
    goalDistance = goalEngine->getInputVariable("goalDist");

    steer = goalEngine->getOutputVariable("steer");
    speed = goalEngine->getOutputVariable("speed");


    //################## Collecting Engine #############################
    collectorEngine = FllImporter().fromFile(
            "../fuzzy_control/Collector.fll");

    std::string status2;
    if (not collectorEngine->isReady(&status))
        throw Exception("[collectorEngine error] collectorEngine is not ready:n" + status2, FL_AT);


    obsDirCol = collectorEngine->getInputVariable("obsDir");
    obsDistCol = collectorEngine->getInputVariable("obsDist");
    marbleDir = collectorEngine->getInputVariable("goalDir");
    marbleDist = collectorEngine->getInputVariable("goalDist");

    collectSteer = collectorEngine->getOutputVariable("steer");
    collectSpeed = collectorEngine->getOutputVariable("speed");

    //################## Freeroam Engine ##################################

    freeroamEngine = FllImporter().fromFile(
            "../fuzzy_control/ObstacleAvoidance.fll");// bemærk et niveau op, kunne også have flyttet .fll

    std::string status3;
    if (not freeroamEngine->isReady(&status3))
        throw Exception("[freeroamEngine error] freeroamEngine is not ready:n" + status3, FL_AT);

    obsDirRoam = freeroamEngine->getInputVariable("obsDir");
    obsDistRoam = freeroamEngine->getInputVariable("obsDist");

    roamSteer = freeroamEngine->getOutputVariable("steer");
    roamSpeed = freeroamEngine->getOutputVariable("speed");


    //################# Draw map of path #################################

    map = cv::imread("../map/smallworld_floor_plan.png");// Load smallworld map, for use in drawing the robots path
    cv::resize(map, mapCopy, cv::Size(resizedWidth, resizedHeight), 0,0,cv::INTER_NEAREST);

    /// Mark start point
    cv::Point2f startPos ((resizedWidth/2+(std::get<0>(currentCoordinates)*combindedResizeFacotor)), (resizedHeight/2-(std::get<1>(currentCoordinates)*combindedResizeFacotor))); //
    cv::circle(mapCopy, startPos, 20,  cv::Scalar(0, 255, 0), -1,8, 0);



}

void FuzzyControl::lidarCallback(ConstLaserScanStampedPtr & msg) {
    //std::cout << ">> " << msg->DebugString() << std::endl;
    float angle_min = float(msg->scan().angle_min());
    double angle_max = msg->scan().angle_max();
    float angle_increment = float(msg->scan().angle_step());

    float range_min = float(msg->scan().range_min());
    float range_max = float(msg->scan().range_max());


    int nranges = msg->scan().ranges_size();
    int nintensities = msg->scan().intensities_size();

    assert(nranges == nintensities);

    for (int i = 0; i < nranges; i++) {

        float angle = angle_min + i * angle_increment;
        float range = std::min(float(msg->scan().ranges(i)), range_max);

        if((range < 10) && (flag))
            {
            lidar_data.push_back(std::tuple<float, float>(angle, range));
            }

        }
    flag=false;
}

#if ENABLE_GLOBAL_POS == 1
void FuzzyControl::poseCallbackNew(ConstPosesStampedPtr & msg)
{
    float x, y, qw, qx, qy, qz;

    mutexFuzzy.lock();
    for (int i = 0; i < msg->pose_size(); i++) {
        if (msg->pose(i).name() == "pioneer2dx") {
            x = msg->pose(i).position().x();
            y = msg->pose(i).position().y();

            // Quaternions
            qw = msg->pose(i).orientation().w();
            qx = msg->pose(i).orientation().x();
            qy = msg->pose(i).orientation().y();
            qz = msg->pose(i).orientation().z(); // seen from x direction a left rotation is positive, and a right is negative TROR JEG

        }


    }

    double siny_cosp = +2.0 *(qw *qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy*qy+qz*qz);
    float rz= atan2(siny_cosp,cosy_cosp);
    currentCoordinates = std::tie(x, y, rz);

//    std::cout << std::setprecision(2) << std::fixed << std::setw(6)
//              << "current x: " << x << std::setw(6)
//              << " current y: " << y << std::setw(6)
//              << " orientation: " << rz << std::setw(6)<< std::endl; // seen from x direction a left rotation is positive, and a right is negative

    mutexFuzzy.unlock();

}
#endif

bool FuzzyControl::move(float &speed2, float &dir) {

    float robX = std::get<0>(currentCoordinates); float robY = std::get<1>(currentCoordinates);
    float goalX = std::get<0>(goalCoordinates); float goalY = std::get<1>(goalCoordinates);
#if DRAW_PATH == 1

    drawRobotActualPath(robX, robY);
    static int print=1;

    if (abs(robX - goalX) < 0.15 && abs(robY - goalY) < 0.15 && print) {
        speed2 = 0;
        std::cout << "HURRA" << std::endl;
        saveRobotPathToFile();
        print--;
    }
#endif

    flag=true;
    while(flag);

    float closest = 10;
    int index = -1;

    for(int i = 0; i < lidar_data.size() ;i++)// finds closest range from lidar scanner
        {
        if (closest > std::get<1>(lidar_data[i]))
            {
            closest = std::get<1>(lidar_data[i]);
            index = i;
            }
        }


    obsDir->setValue(std::get<0>(lidar_data[index]));
    obsDist->setValue(std::get<1>(lidar_data[index]));
#if FREE_ROAM == 0
    if (std::get<0>(goalCoordinates) < std::numeric_limits<float>::max() || std::get<1>(goalCoordinates) < std::numeric_limits<float>::max()) {
        float goalDir, goalDist;
        std::tie(goalDir, goalDist) = calculateGoalDir('r');
        goal->setValue(goalDir);
        goalDistance->setValue(goalDist);
    }

//    float goalDir, goalDist;
//    std::tie(goalDir, goalDist) = calculateGoalDir('r');
//    goal->setValue(goalDir);
//    goalDistance->setValue(goalDist);
#endif

    lidar_data.clear();

    goalEngine->process();
    float dirTmp = steer->getValue();
    float speedTmp = speed->getValue();

    if (!isnan(dirTmp) && !isnan(speedTmp))
    {
        dir = dirTmp;
        speed2 = speedTmp;
    }
//    else {
//        std::cout << "Hov hov du!\n";
//    }

    bool goalReached = false;
    if (abs(robX - goalX) < 0.5 && abs(robY - goalY) < 0.3) {
        goalReached = true;
    }
    return goalReached;
}

bool FuzzyControl::collect(float & speed2, float & dir)
{
    flag = true;
    while(flag);        // Waits for lidars to produce data

    float closest = 10;
    int index = -1;

    for(int i = 0; i < lidar_data.size() ;i++)// finds closest range from lidar scanner
    {
        if (closest > std::get<1>(lidar_data[i]))
        {
            closest = std::get<1>(lidar_data[i]);
            index = i;
        }
    }


    obsDirCol->setValue(std::get<0>(lidar_data[index]));
    obsDistCol->setValue(std::get<1>(lidar_data[index]));

    float goalDir, goalDist;
    std::tie(goalDir, goalDist) = calculateGoalDir('m');

    marbleDir -> setValue(goalDir);
    marbleDist -> setValue(goalDist);

    lidar_data.clear();

    collectorEngine->process();
    float dirTmp = collectSteer->getValue();
    float speedTmp = collectSpeed->getValue();

    if (!isnan(dirTmp) && !isnan(speedTmp))
    {
        dir = dirTmp;
        speed2 = speedTmp;
    }

    bool marbleCollected = false;
    float robX = std::get<0>(currentCoordinates);
    float robY = std::get<1>(currentCoordinates);
    float marbleX = std::get<0>(marbleCoordinates);
    float marbleY = std::get<1>(marbleCoordinates);

    if (abs(robX - marbleX) < 0.1 && abs(robY - marbleY) < 0.1) {
        marbleCollected = true;
    }
    return marbleCollected;

}

void FuzzyControl::freeRoam(float &speed2, float &dir) {

    flag=true;
    while(flag);

    float closest = 10;
    int index = -1;

    for(int i = 0; i < lidar_data.size() ;i++)// finds closest range from lidar scanner
    {
        if (closest > std::get<1>(lidar_data[i]))
        {
            closest = std::get<1>(lidar_data[i]);
            index = i;
        }
    }

    obsDirRoam->setValue(std::get<0>(lidar_data[index]));
    obsDistRoam->setValue(std::get<1>(lidar_data[index]));

    lidar_data.clear();
    freeroamEngine->process();

    float dirTmp = roamSteer->getValue();
    float speedTmp = roamSpeed->getValue();

    if (!isnan(speedTmp))
    {
        speed2 = speedTmp;
    }
    if (!isnan(dirTmp)) {
        dir = dirTmp;
    } else {
        dir = 0;
    }
}

std::tuple<float, float> FuzzyControl::globMarble(const float mDir, const float mDist)
{
    float robX = std::get<0>(currentCoordinates);
    float robY = std::get<1>(currentCoordinates);
    float robAngle = std::get<2>(currentCoordinates);

    // Marble position in robot's local coordinates
    cv::Mat marbleLocal = cv::Mat(2, 1, CV_32FC1);
    marbleLocal.at<float>(0,0) = cos(mDir) * mDist;
    marbleLocal.at<float>(1,0) = sin(mDir) * mDist;

    // The inverse of the robot's rotational matrix
    cv::Mat rotMatrixInverse = cv::Mat(2, 2, CV_32FC1);
    rotMatrixInverse.at<float>(0,0) = cos(robAngle); rotMatrixInverse.at<float>(0,1) = -sin(robAngle);
    rotMatrixInverse.at<float>(1,0) = sin(robAngle); rotMatrixInverse.at<float>(1,1) = cos(robAngle);

    cv::Mat deltaGlob = rotMatrixInverse * marbleLocal;

    float marbleX = deltaGlob.at<float>(0,0) + robX;
    float marbleY = deltaGlob.at<float>(1,0) + robY;

#if OVERRIDE_CV_MARBLE == 0
    return std::tie(marbleX, marbleY);
        //std::cout << "Marble at ( " << marbleX << " , " << marbleY << " )" << std::endl;

#else
    marbleCoordinates = std::tuple<float, float, float> (5, 0, mDist);
    std::cout << "Marble at ( " << 5 << " , " << 0 << " )" << std::endl;


#endif

}

void FuzzyControl::setMarble(float x, float y)
{
    std::get<0>(marbleCoordinates) = x;
    std::get<1>(marbleCoordinates) = y;
}


void FuzzyControl::setGoal(float x, float y)
{
    goalCoordinates = std::tie(x,y);
    /// Mark end point
    cv::Point2f endPos ((resizedWidth/2+(std::get<0>(goalCoordinates)*combindedResizeFacotor)), (resizedHeight/2-(std::get<1>(goalCoordinates)*combindedResizeFacotor))); //
    cv::circle(mapCopy, endPos, 20,  cv::Scalar(0, 0, 255), -1,8, 0);
}

std::tuple<float, float> FuzzyControl::calculateGoalDir(char c)
{
    /// method using atan2:
//    mutexFuzzy.lock();
    float robAngle = std::get<2>(currentCoordinates);
    float robX =std::get<0>(currentCoordinates);
    float robY =std::get<1>(currentCoordinates);
    float goalX, goalY;

//    mutexFuzzy.unlock();
    if (c == 'm'){
        goalX = std::get<0>(marbleCoordinates);
        goalY = std::get<1>(marbleCoordinates);
    }
    else
    {
        goalX = std::get<0>(goalCoordinates);
        goalY = std::get<1>(goalCoordinates);
    }

    float deltaX = goalX - robX;
    float deltaY = goalY - robY;

    cv::Vec2f distRobGoal {deltaX, deltaY};

    cv::Mat rotMatrice = cv::Mat(2, 2, CV_32FC1);
    rotMatrice.at<float>(0,0) = cos(robAngle); rotMatrice.at<float>(0,1) = sin(robAngle);
    rotMatrice.at<float>(1,0) = -sin(robAngle); rotMatrice.at<float>(1,1) = cos(robAngle);

    cv::Mat goalLocal = rotMatrice * cv::Mat(distRobGoal);

    float goalDir = atan2((goalLocal.at<float> (1,0)),(goalLocal.at<float> (0,0)));
    float goalDist = sqrt(deltaX*deltaX + deltaY*deltaY);

    return std::tie(goalDir, goalDist);
}

std::tuple<float, float, float> FuzzyControl::getCoords() {
    return std::tuple<float, float, float>(
            std::get<0>(currentCoordinates),
            std::get<1>(currentCoordinates),
            std::get<2>(currentCoordinates));
}


void FuzzyControl::drawRobotActualPath(float x, float y)
{


    cv::Point2f positionToDraw ((resizedWidth/2+(x*combindedResizeFacotor)), (resizedHeight/2-(y*combindedResizeFacotor))); //


    cv::circle(mapCopy, positionToDraw, 10,  cv::Scalar(255, 0, 0), -1,8, 0);
}

void FuzzyControl::saveRobotPathToFile()
{
    cv::imwrite("../test/test1.png", mapCopy);
}

FuzzyControl::~FuzzyControl() {

    lidar_data.clear();

}

void FuzzyControl::drawRobotActualPath2(float x, float y, cv::Mat& map, cv::Vec3b color)
{


    cv::Point2f positionToDraw ((resizedWidth/2+(x*combindedResizeFacotor)), (resizedHeight/2-(y*combindedResizeFacotor))); //


    cv::circle(map, positionToDraw, 2, color, -1,8, 0);
}








