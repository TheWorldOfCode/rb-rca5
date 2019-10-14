#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include "fl/Headers.h"

#include <iostream>
#include <vector>
#include <tuple>

static boost::mutex mutex;

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

void cameraCallback(ConstImageStampedPtr &msg) {

  std::size_t width = msg->image().width();
  std::size_t height = msg->image().height();
  const char *data = msg->image().data().c_str();
  cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

  im = im.clone();
  cv::cvtColor(im, im, CV_RGB2BGR);

  mutex.lock();
  cv::imshow("camera", im);
  mutex.unlock();
}
// new is a temp name
void cameraCallbackNew(ConstImageStampedPtr &msg) {

    std::size_t width = msg->image().width();
    std::size_t height = msg->image().height();
    const char *data = msg->image().data().c_str();
    cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

    im = im.clone();



    cv::cvtColor(im, im, CV_RGB2GRAY);

    GaussianBlur(im, im, cv::Size(9, 9), 2, 0);

    ////
    std::vector<cv::Vec3f> circles;
    static cv::Point current(-1, -1);

    /// Apply the Hough Transform to find the circles
    //
    HoughCircles(im, circles, CV_HOUGH_GRADIENT,
                 1,   // accumulator resolution (size of the image / 2)
                 3000,  // minimum distance between two circles
                 20, // Canny high threshold
                 30, // minimum number of votes
                 0, 0); // min and max radius 8 15

    /// Draw the circles detected
    for (size_t i = 0; i < circles.size(); i++)
        {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle(im, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
        // circle outline
        circle(im, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);

//        if (current != center)
//            {
//            std::cout << "Position of the white ball is:" << center << std::endl; //
//            current = center;
//            }
        }
    ////

    mutex.lock();
    cv::imshow("New camera", im);// temp name
    mutex.unlock();
}

std::vector<std::tuple<float,float>> test;//////////
bool flag = false;
void lidar_fuzzy_callback(ConstLaserScanStampedPtr &msg) {

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
    if((range < 3) && (flag))
        {
        test.push_back(std::tuple<float, float>(angle, range));
        }
    //////

  }
  flag=false;
}
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

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo topics
  gazebo::transport::SubscriberPtr statSubscriber =
      node->Subscribe("~/world_stats", statCallback);

//  gazebo::transport::SubscriberPtr poseSubscriber =
//      node->Subscribe("~/pose/info", poseCallback);

  gazebo::transport::SubscriberPtr cameraSubscriber =
      node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallback);
// new subscriber
    gazebo::transport::SubscriberPtr cameraSubscriberNew =
            node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallbackNew);

  gazebo::transport::SubscriberPtr lidarSubscriber =
      node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallback);

    gazebo::transport::SubscriberPtr lidar_fuzzy_Subscriber =
            node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidar_fuzzy_callback);

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

  const int key_left = 81;
  const int key_up = 82;
  const int key_down = 84;
  const int key_right = 83;
  const int key_esc = 27;

  float speed = 0.0;//////////////////////
  float dir = 0.0;

  // Loop

  ///////////////////
    using namespace fl;
    Engine* engine = FllImporter().fromFile("../fuzzy_control/ObstacleAvoidance.fll");// bemærk et niveau op, kunne også have flyttet .fll

    std::string status;
    if (not engine->isReady(&status))
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);

    InputVariable* obstacle = engine->getInputVariable("obstacle");
    InputVariable* goal = engine->getInputVariable("goal");
    OutputVariable* steer = engine->getOutputVariable("mSteer");







  //////////////////
  while (true) {

      //// start of fuzzy controller
      flag=true;
      while(flag);

      float closest=10;
      int index=-1;
      for(int i=0; i<test.size() ;i++)// finds closest range from lidar scanner
          {
             if(closest>std::get<1>(test[i]))
                 {
                 closest=std::get<1>(test[i]);
                 index=i;
                 }
          }




      obstacle->setValue(std::get<0>(test[index]));
      std::cout<< "angle: "<< std::get<0>(test[index]) <<std::endl;
      test.clear();

      engine->process();
      dir = steer->getValue();

      std::cout << "output dir " << dir << std::endl;
///// end of fuzzy controller


      gazebo::common::Time::MSleep(10);
//
    mutex.lock();
    int key = cv::waitKey(1);
    mutex.unlock();
//
//    if (key == key_esc)
//      break;
//
//    if ((key == key_up) && (speed <= 1.2f))
//      speed += 0.05;
//    else if ((key == key_down) && (speed >= -1.2f))
//      speed -= 0.05;
//    else if ((key == key_right) && (dir <= 0.4f))
//      dir += 0.05;
//    else if ((key == key_left) && (dir >= -0.4f))
//      dir -= 0.05;
//    else {
//      // slow down
//      //      speed *= 0.1;
//      //      dir *= 0.1;
//    }

    // Generate a pose
    ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

    // Convert to a pose message
    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);
    movementPublisher->Publish(msg);
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
