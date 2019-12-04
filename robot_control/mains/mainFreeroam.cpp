#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <tuple>

#include "../includes/FuzzyControl.h"
#include "../includes/Vision.h"
#include "../includes/line_detect.hpp"


static boost::mutex mutex; /// copied to Vision, but is also needed to run lidar from main

void statCallback(ConstWorldStatisticsPtr &_msg) {
    (void)_msg;
}

void poseCallback(ConstPosesStampedPtr &_msg) {
    for (int i = 0; i < _msg->pose_size(); i++) {
        if (_msg->pose(i).name() == "pioneer2dx") {

            std::cout << std::setprecision(2) << std::fixed << std::setw(6)
                      << _msg->pose(i).position().x() << std::setw(6)
                      << _msg->pose(i).position().y() << std::setw(6)
                      << _msg->pose(i).position().z() << std::setw(6)
                      << _msg->pose(i).orientation().w() << std::setw(6)
                      << _msg->pose(i).orientation().x() << std::setw(6)
                      << _msg->pose(i).orientation().y() << std::setw(6)
                      << _msg->pose(i).orientation().z() << std::endl;
        }
    }
}

void lidarCallback(ConstLaserScanStampedPtr &msg) {

    float angle_min = float(msg->scan().angle_min());
    float angle_increment = float(msg->scan().angle_step());

    float range_min = float(msg->scan().range_min());
    float range_max = float(msg->scan().range_max());

    int sec = msg->time().sec();
    int nsec = msg->time().nsec();

    int nranges = msg->scan().ranges_size();
    int nintensities = msg->scan().intensities_size();

    assert(nranges == nintensities);

    int width = 400;
    int height = 400;
    float px_per_m = 200 / range_max;

    cv::Mat im(height, width, CV_8UC3);
    im.setTo(0);
    for (int i = 0; i < nranges; i++) {
        float angle = angle_min + i * angle_increment;
        float range = std::min(float(msg->scan().ranges(i)), range_max);

        cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                            200.5f - range_min * px_per_m * std::sin(angle));
        cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                          200.5f - range * px_per_m * std::sin(angle));
        cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
                 cv::LINE_AA, 4);
    }

    cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
    cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
                cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
                cv::Scalar(255, 0, 0));
}


int main(int _argc, char **_argv) {
    // Load gazebo
    gazebo::client::setup(_argc, _argv);

    FuzzyControl controller;

    Vision camera;

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo topics
    gazebo::transport::SubscriberPtr statSubscriber =
            node->Subscribe("~/world_stats", statCallback);

    // creates lidar subscriber
    gazebo::transport::SubscriberPtr lidarSubscriber =
            node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallback);

    // creates subscriber to FuzzyControl
    gazebo::transport::SubscriberPtr lidar_fuzzy_Subscriber =
            node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", &FuzzyControl::lidarCallback, &controller);

    // creates subscriber to current pose
    gazebo::transport::SubscriberPtr poseSubscriber =
            node->Subscribe("~/pose/info", &FuzzyControl::poseCallbackNew, & controller);

    // creates subscriber to camera
    gazebo::transport::SubscriberPtr cameraSubscriber =
            node->Subscribe("~/pioneer2dx/camera/link/camera/image", &Vision::cameraCallbackRaw, &camera);

    // Publish to the robot vel_cmd topic
    gazebo::transport::PublisherPtr movementPublisher =
            node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

    // Publish a reset of the world
    gazebo::transport::PublisherPtr worldPublisher =
            node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
    gazebo::msgs::WorldControl controlMessage;
    controlMessage.mutable_reset()->set_all(true);
    worldPublisher->WaitForConnection();
    worldPublisher->Publish(controlMessage);

    float speed = 0.0;
    float dir = 0.0;

    // Loop
    while (true) {

        gazebo::common::Time::MSleep(10);     //
        mutex.lock();
        int key = cv::waitKey(1);
        mutex.unlock();

        ///////////////////////////////////////
        controller.freeRoam(speed, dir);
        ///////////////////////////////////////

        // Generate a pose
        ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

        // Convert to a pose message
        gazebo::msgs::Pose msg;
        gazebo::msgs::Set(&msg, pose);
        movementPublisher->Publish(msg);
    }
}

