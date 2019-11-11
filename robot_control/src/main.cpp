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

#define AUTO_MOVE 1


#define IGNORE_MARBLE 0


#include "../includes/line.hpp"


static boost::mutex mutex; /// copied to Vision, but is also needed to run lidar from main

void statCallback(ConstWorldStatisticsPtr &_msg) {
    (void)_msg;
    // Dump the message contents to stdout.
    //  std::cout << _msg->DebugString();
    //  std::cout << std::flush;
}

void poseCallback(ConstPosesStampedPtr &_msg) {
    // Dump the message contents to stdout.
    //  std::cout << _msg->DebugString();

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


//void cameraCallback(ConstImageStampedPtr &msg) {
//
//  std::size_t width = msg->image().width();
//  std::size_t height = msg->image().height();
//  const char *data = msg->image().data().c_str();
//  cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));
//
//  im = im.clone();
//  cv::cvtColor(im, im, CV_RGB2BGR);
//
//  mutex.lock();
//  cv::imshow("camera", im);
//  mutex.unlock();
//}
// new is a temp name
//void cameraCallbackNew(ConstImageStampedPtr &msg) {
//
//    std::size_t width = msg->image().width();
//    std::size_t height = msg->image().height();
//    const char *data = msg->image().data().c_str();
//    cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));
//
//    im = im.clone();
//
//
//
//    cv::cvtColor(im, im, CV_RGB2GRAY);
//
//    GaussianBlur(im, im, cv::Size(9, 9), 2, 0);
//
//    ////
//    std::vector<cv::Vec3f> circles;
//    static cv::Point current(-1, -1);
//
//    /// Apply the Hough Transform to find the circles
//    //
//    HoughCircles(im, circles, CV_HOUGH_GRADIENT,
//                 1,   // accumulator resolution (size of the image / 2)
//                 3000,  // minimum distance between two circles
//                 20, // Canny high threshold
//                 30, // minimum number of votes
//                 0, 0); // min and max radius 8 15
//
//    /// Draw the circles detected
//    for (size_t i = 0; i < circles.size(); i++)
//        {
//        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//        int radius = cvRound(circles[i][2]);
//        // circle center
//        circle(im, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
//        // circle outline
//        circle(im, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
//
////        if (current != center)
////            {
////            std::cout << "Position of the white ball is:" << center << std::endl; //
////            current = center;
////            }
//        }
//    ////
//
//    mutex.lock();
//    cv::imshow("New camera", im);// temp name
//    mutex.unlock();
//}

void lidarCallback(ConstLaserScanStampedPtr &msg) {

    //  std::cout << ">> " << msg->DebugString() << std::endl;
    float angle_min = float(msg->scan().angle_min());
    //  double angle_max = msg->scan().angle_max();
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
        //////

        //////
        //    double intensity = msg->scan().intensities(i);
        cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                            200.5f - range_min * px_per_m * std::sin(angle));
        cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                          200.5f - range * px_per_m * std::sin(angle));
        cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
                 cv::LINE_AA, 4);

        //    std::cout << angle << " " << range << " " << intensity << std::endl;
        }

    cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
    cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
                cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
                cv::Scalar(255, 0, 0));

    mutex.lock();
    cv::imshow("lidar", im);
    mutex.unlock();
}


int main(int _argc, char **_argv) {
    // Load gazebo
    gazebo::client::setup(_argc, _argv);

    FuzzyControl controller;

    Vision camera;//////////////////////////

#if DEBUG_LINE_DETECT == 1
    line_detect::line_detect_test lineDetectTest(1) ;
#endif
    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo topics
    gazebo::transport::SubscriberPtr statSubscriber =
            node->Subscribe("~/world_stats", statCallback);

    //  gazebo::transport::SubscriberPtr poseSubscriber =
    //      node->Subscribe("~/pose/info", poseCallback);

    //  gazebo::transport::SubscriberPtr cameraSubscriber =
    //      node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallback);
    //// new subscriber
    //    gazebo::transport::SubscriberPtr cameraSubscriberNew =
    //            node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallbackNew);
    // creates lidar subscriber
    gazebo::transport::SubscriberPtr lidarSubscriber =
            node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallback);

    // creates subscriber to FuzzyControl
    gazebo::transport::SubscriberPtr lidar_fuzzy_Subscriber =
            node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", &FuzzyControl::lidarCallback, &controller);

    // CHEAT subscriber to current pose
    gazebo::transport::SubscriberPtr poseSubscriber =
            node->Subscribe("~/pose/info", &FuzzyControl::poseCallbackNew, & controller);

    // creates subscriber to camera, with no magic
    gazebo::transport::SubscriberPtr cameraSubscriber =
            node->Subscribe("~/pioneer2dx/camera/link/camera/image", &Vision::cameraCallbackRaw, &camera);

    // creates subscriber to camera, with Hough circle transform
    gazebo::transport::SubscriberPtr cameraSubscriberHough =
            node->Subscribe("~/pioneer2dx/camera/link/camera/image", &Vision::cameraCallbackHough, &camera);



#if DEBUG_LINE_DETECT == 1
    gazebo::transport::SubscriberPtr lineDetectTestSub =
		node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", &line_detect::line_detect_test::lidarCallback, &lineDetectTest );
#endif

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

#if DEBUG_LINE_DETECT == 1
    /*vec2 a(3, 2);
        vec2 b(3, 3);
	vec2 r;
	double r2;

	std::cout << "a: " << a << " b: " << b << std::endl;
	r = a + b;
	std::cout << "Adding: "  << r << std::endl;
	r = a - b;
	std::cout << "Sub: " << r << std::endl;
	r2 = a * b;
	std::cout << "Cross: " << r2 << std::endl;
	r = a * 2.0;
        std::cout << "scalar: "  << r << std::endl;

        vec2 p3(5,6);

	line l(2,2,4,4, pointType::CARTESIAN);
	std::cout << "line " << l << std::endl;
        std::cout << "Distance to " << p3 << " " l.distance_to_point(p3) << std::endl;
	*/

	lineDetectTest.wait_on_data();
	for(double i = 0.1; i <= 0.17; i+= 0.00001 )
	{
              lineDetectTest.split_and_merge_show_steps(i);
	}
# else
    const int key_left = 81;
    const int key_up = 82;
    const int key_down = 84;
    const int key_right = 83;
    const int key_esc = 27;
#if AUTO_MOVE ==0
    float speed = 0.0;
    float dir = 0.0;
# else
    float speed = 0.3;
    float dir = 0.0;
#endif

    controller.setGoal(-4,-1);

    float marbleDir, marbleDist;
    bool marbleFound;
    bool collectMode = false;
    bool collectDone;
    const int marbleDetectionLimit = 4;
    int marbleDetections = 0;
    float avgMarbleDir = 0, avgMarbleDist = 0;

    // Loop
    while (true) {

        gazebo::common::Time::MSleep(10);     //
        mutex.lock();
        int key = cv::waitKey(1);
        mutex.unlock();

#if AUTO_MOVE ==1
#if IGNORE_MARBLE == 1
        controller.move(speed, dir);
#else

        marbleFound = false;
        std::tie(marbleFound, marbleDir, marbleDist) = camera.getMarble();

        if ( marbleFound ) {
            if (marbleDetections < marbleDetectionLimit) {
                marbleDetections++;
                avgMarbleDir += marbleDir;
                avgMarbleDist += marbleDist;
                std::cout << "Marble detections: " << marbleDetections << std::endl;
                std::cout << "MarbleDir:    " << marbleDir << "\tMarbleDist:    " << marbleDist << std::endl;
            }
            if (marbleDetections == marbleDetectionLimit) {
                marbleDetections++;
                avgMarbleDir /= float(marbleDetectionLimit);
                avgMarbleDist /= float(marbleDetectionLimit);
                std::cout << "avgMarbleDir: " << avgMarbleDir << "\tavgMarbleDist: " << avgMarbleDist << std::endl;

                controller.setMarble(avgMarbleDir, avgMarbleDist);
                collectMode = true; std::cout << "collectMode == true" << std::endl;
            }

        }

//
//        if (marbleFound)
//            {
//            collectMode = true;
//            controller.setMarble(marbleDir, marbleDist);
//            }

        if (collectMode)
        {
            collectDone = controller.collect(speed, dir);
            if (collectDone) {
                collectMode = false; std::cout << "collectMode == false" << std::endl;
                avgMarbleDir = 0;
                avgMarbleDist = 0;
                marbleDetections = 0;
            }
        }
        else
        {
            controller.move(speed, dir);
        }
#endif
# else

        if (key == key_esc)
            break;

        if ((key == key_up) && (speed <= 1.2f))
            speed += 0.05;
        else if ((key == key_down) && (speed >= -1.2f))
            speed -= 0.05;
        else if ((key == key_right) && (dir <= 0.4f))
            dir += 0.05;
        else if ((key == key_left) && (dir >= -0.4f))
            dir -= 0.05;
        //else {
            // slow down
            //      speed *= 0.1;
            //      dir *= 0.1;
          //}
#endif


        // Generate a pose
        ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

        // Convert to a pose message
        gazebo::msgs::Pose msg;
        gazebo::msgs::Set(&msg, pose);
        movementPublisher->Publish(msg);
        }
#endif
    // Make sure to shut everything down.
    gazebo::client::shutdown();
}
