//
// Created by lars on 10/14/19.
//
#define FUZZY_DEBUG 0


#include <string>
#include <cmath>        // std::atan2

#if FUZZY_DEBUG == 1
#include <iostram>
#endif

#include "../includes/FuzzyControl.h"

using namespace fl;

FuzzyControl::FuzzyControl() {
    flag = false;

    engine = FllImporter().fromFile(
//            "../fuzzy_control/ObstacleAvoidanceWorking.fll");// bemærk et niveau op, kunne også have flyttet .fll
            "../fuzzy_control/playground.fll");// bemærk et niveau op, kunne også have flyttet .fll

    std::string status;
    if (not engine->isReady(&status))
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);

    obsDir = engine->getInputVariable("obsDir");
    obsDist = engine->getInputVariable("obsDist");
    goal = engine->getInputVariable("goal");

    steer = engine->getOutputVariable("steer");
    speed = engine->getOutputVariable("speed");
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

        if((range < 3) && (flag))
            {
            lidar_data.push_back(std::tuple<float, float>(angle, range));
            }

        }
    flag=false;

}

void FuzzyControl::move(float &speed2, float &dir) {

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

    float goalDir=calculateGoalDir();

    goal -> setValue(goalDir);////

    //std::cout << " obsDist: "<< std::get<1>(lidar_data[index]) << std::endl;
   // std::cout << " obsDir: "<< std::get<0>(lidar_data[index]) << std::endl;





#if FUZZY_DEBUG == 1
    std::cout<< "angle: "<< std::get<0>(lidar_data[index]) <<std::endl;
#endif

    lidar_data.clear();

    engine->process();
    dir = steer->getValue();
    speed2 = speed->getValue();

#if FUZZY_DEBUG == 1
    std::cout << "output dir " << dir << std::endl;
#endif

}

void FuzzyControl::setGoal(float x, float y)
{
  goalCoordinates = std::tie(x,y);
}

#if ENABLE_GLOBAL_POS == 1
void FuzzyControl::poseCallbackNew(ConstPosesStampedPtr & msg)
{
    mutexFuzzy.lock();
    for (int i = 0; i < msg->pose_size(); i++) {
        if (msg->pose(i).name() == "pioneer2dx") {
             float x = msg->pose(i).position().x();
             float y = msg->pose(i).position().y();
             float rz = msg->pose(i).orientation().z(); // seen from x direction a left rotation is positive, and a right is negative TROR JEG

            std::cout << std::setprecision(2) << std::fixed << std::setw(6)
                      << "current x: " << x << std::setw(6)
                      << " current zy: " << y << std::setw(6)
                      << " orientation: " << rz << std::setw(6)<< std::endl; // seen from x direction a left rotation is positive, and a right is negative

            currentCoordinates = std::tie(x, y, rz);
            }
        }
    mutexFuzzy.unlock();

}
#endif

float FuzzyControl::calculateGoalDir()
{
    /// method using atan2:
   // mutexFuzzy.lock();
    float robAngle = std::get<2>(currentCoordinates);
    float robX =std::get<0>(currentCoordinates);
    float robY =std::get<1>(currentCoordinates);
   // mutexFuzzy.unlock();

    cv::Mat goalTrans = cv::Mat(3,3,CV_32FC1);

    goalTrans.at<float>(0,0)=cos(robAngle); goalTrans.at<float>(0,1)=sin(robAngle); goalTrans.at<float>(0,2)=-robX;

    goalTrans.at<float>(1,0)=-sin(robAngle); goalTrans.at<float>(1,1)=cos(robAngle); goalTrans.at<float>(1,2)=-robY;

    goalTrans.at<float>(2,0)=float(0); goalTrans.at<float>(2,1)=float(0); goalTrans.at<float>(2,2)=float(1);

    cv::Mat goalGlobal = cv::Mat(3,1,CV_32FC1);
    goalGlobal.at<float>(0,0)=std::get<0>(goalCoordinates);
    goalGlobal.at<float>(1,0)=std::get<1>(goalCoordinates);
    goalGlobal.at<float>(2,0)=float(1);

//    cv::Mat goalGlobal = {std::get<0>(goalCoordinates),std::get<1>(goalCoordinates),float(1)};

    cv::Mat goalLocal = goalTrans * goalGlobal;

    float goalDir = atan2((goalLocal.at<float> (1,0)),(goalLocal.at<float> (0,0)));


        std::cout << " goal global X: " << std::get<0>(goalCoordinates) << " goal global Y: " << std::get<1>(goalCoordinates)<<std::endl;

        std::cout << " rob global X: " << robX << " rob global Y: " << robY<< " rob angle: "<< robAngle << std::endl;

        std::cout << " goal Local X: "<< goalLocal.at<float> (0,0)<< " goalLocal Y: "<< goalLocal.at<float> (1,0) << std::endl;

        std::cout << " goalDir: "<< goalDir << std::endl;



//    float deltaX=std::get<0>(goalCoordinates) - std::get<0>(currentCoordinates);
//
//    float deltaXAbs = fabs(deltaX);
//
//    float deltaY = std::get<1>(goalCoordinates) - std::get<1>(currentCoordinates);
//
//    float goalDirTmp = (std::atan2(deltaY, deltaX)) - std::get<2>(currentCoordinates);
//
//
//    std::cout << " deltaX: "<< deltaX<< " deltaY: "<< deltaY << " goalDirTmp: " << goalDirTmp << std::endl;
//
//
//    if( (-3<goalDirTmp<3))
//        {
//        goalDir=goalDirTmp;
//        }
//    else
//        {
//        std::cout << "swappin!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
//        goalDir = 3;
////        goalDir = goalDirTmp-3.14;
////        goalDir =std::get<2>(currentCoordinates) - (std::atan2(deltaY, deltaXAbs));
//        }



    /// method using acos:
//    float deltaX=std::get<0>(goalCoordinates) - std::get<0>(currentCoordinates);
//    float deltaY = std::get<1>(goalCoordinates) - std::get<1>(currentCoordinates);
//
//    float hypotenuse = sqrtf((deltaX*deltaX)+(deltaY*deltaY));
//    float goalDir;
//
//    std::cout << " deltaX: "<< deltaX<< " deltaY: "<< deltaY<< " hypotenuse: " << hypotenuse << std::endl;
//    if((deltaX>0) && (hypotenuse>0))
//        {
//        goalDir = (acosf(deltaX / hypotenuse)) - std::get<2>(currentCoordinates);
//        }
//    else
//        {
//        goalDir = (acosf(deltaX / hypotenuse)) - std::get<2>(currentCoordinates)+(3.1416);
//        }

    return goalDir;


}

FuzzyControl::~FuzzyControl() {

    lidar_data.clear();

}



