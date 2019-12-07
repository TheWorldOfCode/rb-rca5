#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <tuple>

#include "../includes/FuzzyControl.h"
#include "../includes/Vision.h"


static boost::mutex mutex; /// copied to Vision, but is also needed to run lidar from main

void statCallback(ConstWorldStatisticsPtr &_msg) {
    (void)_msg;
}

std::vector<std::tuple<double, double>> marbles_loc;
void poseCallback(ConstPosesStampedPtr &_msg) {
    for (int i = 0; i < _msg->pose_size(); i++) {
        if (_msg->pose(i).name() == "marble_clone_0") {
            marbles_loc[0] = std::tuple<double,double>(_msg->pose(i).position().x(), _msg->pose(i).position().y());
        }
        if (_msg->pose(i).name() == "marble_clone_1") {
            marbles_loc[1] = std::tuple<double,double>(_msg->pose(i).position().x(), _msg->pose(i).position().y());
        }
        if (_msg->pose(i).name() == "marble_clone_2") {
            marbles_loc[2] = std::tuple<double,double>(_msg->pose(i).position().x(), _msg->pose(i).position().y());
        }
        if (_msg->pose(i).name() == "marble_clone_3") {
            marbles_loc[3] = std::tuple<double,double>(_msg->pose(i).position().x(), _msg->pose(i).position().y());
        }
        if (_msg->pose(i).name() == "marble_clone_4") {
            marbles_loc[4] = std::tuple<double,double>(_msg->pose(i).position().x(), _msg->pose(i).position().y());
        }
        if (_msg->pose(i).name() == "marble_clone_5") {
            marbles_loc[5] = std::tuple<double,double>(_msg->pose(i).position().x(), _msg->pose(i).position().y());
        }
        if (_msg->pose(i).name() == "marble_clone_6") {
            marbles_loc[6] = std::tuple<double,double>(_msg->pose(i).position().x(), _msg->pose(i).position().y());
        }
        if (_msg->pose(i).name() == "marble_clone_7") {
            marbles_loc[7] = std::tuple<double,double>(_msg->pose(i).position().x(), _msg->pose(i).position().y());
        }
        if (_msg->pose(i).name() == "marble_clone_8") {
            marbles_loc[8] = std::tuple<double,double>(_msg->pose(i).position().x(), _msg->pose(i).position().y());
        }
        if (_msg->pose(i).name() == "marble_clone_9") {
            marbles_loc[9] = std::tuple<double,double>(_msg->pose(i).position().x(), _msg->pose(i).position().y());
        }
        if (_msg->pose(i).name() == "marble_clone_10") {
            marbles_loc[10] = std::tuple<double,double>(_msg->pose(i).position().x(), _msg->pose(i).position().y());
        }
        if (_msg->pose(i).name() == "marble_clone_11") {
            marbles_loc[11] = std::tuple<double,double>(_msg->pose(i).position().x(), _msg->pose(i).position().y());
        }
        if (_msg->pose(i).name() == "marble_clone_12") {
            marbles_loc[12] = std::tuple<double,double>(_msg->pose(i).position().x(), _msg->pose(i).position().y());
        }
        if (_msg->pose(i).name() == "marble_clone_13") {
            marbles_loc[13] = std::tuple<double,double>(_msg->pose(i).position().x(), _msg->pose(i).position().y());
        }
        if (_msg->pose(i).name() == "marble_clone_14") {
            marbles_loc[14] = std::tuple<double,double>(_msg->pose(i).position().x(), _msg->pose(i).position().y());
        }
        if (_msg->pose(i).name() == "marble_clone_15") {
            marbles_loc[15] = std::tuple<double,double>(_msg->pose(i).position().x(), _msg->pose(i).position().y());
        }
        if (_msg->pose(i).name() == "marble_clone_16") {
            marbles_loc[16] = std::tuple<double,double>(_msg->pose(i).position().x(), _msg->pose(i).position().y());
        }
        if (_msg->pose(i).name() == "marble_clone_17") {
            marbles_loc[17] = std::tuple<double,double>(_msg->pose(i).position().x(), _msg->pose(i).position().y());
        }
        if (_msg->pose(i).name() == "marble_clone_18") {
            marbles_loc[18] = std::tuple<double,double>(_msg->pose(i).position().x(), _msg->pose(i).position().y());
        }
        if (_msg->pose(i).name() == "marble_clone_19") {
            marbles_loc[19] = std::tuple<double,double>(_msg->pose(i).position().x(), _msg->pose(i).position().y());
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

    // creates subscriber to camera with Hough circle transform
    gazebo::transport::SubscriberPtr cameraSubscriberHough =
            node->Subscribe("~/pioneer2dx/camera/link/camera/image", &Vision::cameraCallbackHough, &camera);

    // Marbles pos subscriber
    marbles_loc.resize(20);
    gazebo::transport::SubscriberPtr poseSubscriber2 =
            node->Subscribe("~/pose/info", poseCallback);

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

    float marbleDir, marbleDist;
    bool marbleFound;
    bool collectMode = false;
    bool collectDone;
    const int marbleDetectionLimit = 3;
    int marbleDetections = 0;
    float mX, mY, mXnew, mYnew;
    float avgMarbleX = 0, avgMarbleY = 0;

    float robX, robY, robA;
    float scaleFromPictureToModel =1/1.41735;
    float resizeFactor = 15;
    float combindedResizeFacotor=scaleFromPictureToModel*resizeFactor;
    float resizedWidth =120*combindedResizeFacotor*scaleFromPictureToModel; // width meaning x
    float resizedHeight =80*combindedResizeFacotor*scaleFromPictureToModel;
    cv::Mat map = cv::imread("../map/bigworld_floor_plan.png");// Load smallworld map, for use in drawing the robots path
    cv::resize(map, map, cv::Size(resizedWidth, resizedHeight), 0,0,cv::INTER_NEAREST);

    // Loop
    while (true) {

        gazebo::common::Time::MSleep(10);     //
        mutex.lock();
        int key = cv::waitKey(1);
        mutex.unlock();

        ///////////////////////////////////////
        marbleFound = false;
        /// Marble detections will happen in the callback functions.
        std::tie(marbleFound, marbleDir, marbleDist) = camera.getMarble();

        if ( marbleFound ) {
            if ( marbleDetections < marbleDetectionLimit ) {
                /// Calculate global coordinates for the detected marble
                std::tie(mXnew, mYnew) = controller.globMarble(marbleDir, marbleDist);

                if ( marbleDetections == 0 ) {
                    mX = mXnew; mY = mYnew;
                }

                /// If the detected marble is close to the previous detection, we begin calculating average position
                if ( abs(mXnew - mX) < 0.8 && abs(mYnew - mY) < 0.8 ) {
                    marbleDetections++;
                    avgMarbleX += mXnew;
                    avgMarbleY += mYnew;

                    std::cout << "Marble detection " << marbleDetections << " / " << marbleDetectionLimit << "\n";
                    std::cout << "at (" << mXnew << " , " << mYnew << ")\n\n";
                }
                else {
                    marbleDetections = 0;
                    avgMarbleX = 0; avgMarbleY = 0;
                }
                mX = mXnew; mY = mYnew;
            }
            /// If enough close detections is made, we set the average as goal and enter collect-mode.
            if ( marbleDetections == marbleDetectionLimit ) {
                marbleDetections++;
                avgMarbleX /= float(marbleDetectionLimit);
                avgMarbleY /= float(marbleDetectionLimit);

                controller.setMarble(avgMarbleX, avgMarbleY);
                collectMode = true;
                std::cout << "Marble confirmed at (" << avgMarbleX << " , " << avgMarbleY << ")\n";
                std::cout << "Collecting...";
            }
        }
        /// In collectMode, the fuzzy collector-controller takes over until the marble is collected.
        if ( collectMode ) {
            collectDone = controller.collect(speed, dir);
            if ( collectDone ) {
                collectMode = false;
                avgMarbleX = 0; avgMarbleY = 0; marbleDetections = 0;
                std::cout << "Marble collected\n\n";
            }
        } else {
            controller.freeRoam(speed, dir);
        }

        /// Print fuzzy path
        std::tie(robX, robY, robA) = controller.getCoords();
        controller.drawRobotActualPath2(robX, robY, map);
        for (int i = 0; i < marbles_loc.size(); i++) {
            //std::cout << "(" << std::get<0>(marbles_loc[i]) << " , " << std::get<1>(marbles_loc[i]) << ")\n";
            cv::Point2f positionToDraw((resizedWidth / 2 + (std::get<0>(marbles_loc[i]) * combindedResizeFacotor)),
                                       (resizedHeight / 2 - (std::get<1>(marbles_loc[i]) * combindedResizeFacotor))); //
            cv::circle(map, positionToDraw, 5, cv::Scalar(0, 0, 255), -1, 8, 0);
        }
        imshow("Path", map);
        if (key == 27) {
            cv::imwrite("../test/TestPaths/FuzzyRoam_Path.png", map);
            break;
        }

        ///////////////////////////////////////

        // Generate a pose
        ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

        // Convert to a pose message
        gazebo::msgs::Pose msg;
        gazebo::msgs::Set(&msg, pose);
        movementPublisher->Publish(msg);
    }
    // Make sure to shut everything down.
    gazebo::client::shutdown();
    return 0;
}

