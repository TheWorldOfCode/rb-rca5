//
// Created by lars on 10/14/19.
//
#define FUZZY_DEBUG 0


#include <string>
#if FUZZY_DEBUG == 1
#include <iostram>
#endif

#include "../includes/FuzzyControl.h"

using namespace fl;

FuzzyControl::FuzzyControl() {
    flag = false;

    engine = FllImporter().fromFile(
            "../fuzzy_control/ObstacleAvoidanceWorking.fll");// bemærk et niveau op, kunne også have flyttet .fll

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

FuzzyControl::~FuzzyControl() {

    lidar_data.clear();

}
