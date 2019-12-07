#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write
#include <iostream>
#include <vector>
#include <fstream>
#include <tuple>
#include <unistd.h> 
#include <time.h>

#include "../includes/FuzzyControl.h"
#include "../includes/Vision.h"
#include "../includes/line_detect.hpp"
#include "../includes/particalfilter.hpp" 
#include "../includes/draw_rute.hpp" 

#define AUTO_MOVE 1


#define IGNORE_MARBLE 0


#include "../includes/line.hpp"

double pos_rob_x = 0;
double pos_rob_y = 0;
double rob_theta = 0;

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

			pos_rob_x = _msg->pose(i).position().x();  
			pos_rob_y = _msg->pose(i).position().y();  
			rob_theta = _msg->pose(i).orientation().z();   
			/*std::cout << std::setprecision(2) << std::fixed << std::setw(6)
			  << _msg->pose(i).position().x() << std::setw(6)
			  << _msg->pose(i).position().y() << std::setw(6)
			  << _msg->pose(i).position().z() << std::setw(6)
			  << _msg->pose(i).orientation().w() << std::setw(6)
			  << _msg->pose(i).orientation().x() << std::setw(6)
			  << _msg->pose(i).orientation().y() << std::setw(6)
			  << _msg->pose(i).orientation().z() << std::endl;
			  }*/
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
//
bool flag = false;
std::vector<std::tuple<double,double>> data;
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
	while(flag); 
	flag = true;
	data.clear(); 
	for (int i = 0; i < nranges; i++) {
		float angle = angle_min + i * angle_increment;
		float range = std::min(float(msg->scan().ranges(i)), range_max);
		//////
		data.push_back(std::tuple<double,double>(-1*angle * 180/3.14,range)); 

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

	flag = false;
	cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
	cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
			cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
			cv::Scalar(255, 0, 0));
	// cv::namedWindow("lidar"); cv::moveWindow("lidar", 0, 0);
	//    mutex.lock();
	//    cv::imshow("lidar", im);
	//    mutex.unlock();
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

	gazebo::transport::SubscriberPtr poseSubscriber2 =
		node->Subscribe("~/pose/info", poseCallback);

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
# elif DEBUG_PARTICLEFILTER > 0
	std::cout << "debugger particle filter"  << std::endl; 
	cv::Mat map = cv::imread("../map/bigworld_floor_plan.png");  

	cv::resize(map, map, cv::Size(), 1/1.41735 * 10, 1/1.41735 * 10); 
	cv::imwrite("../test/localization/orignalMap.png", map );


	const int map_offset_x = map.cols/2;
	const int map_offset_y = map.rows/2 - 5;
	const double MeterPrPixel = 0.1;
	const int Max_Meter = 10;

	const double sigmaData = 0.05;

	ParticleFilter particlefilter(map_offset_x, map_offset_y  ) ;
//	particlefilter.resize(map,map, 72.0/25.4, 1); 

	cv::imwrite("../test/localization/newMap.png", map );
	std::vector<cv::String> lookup_files; 
 	cv::glob("../particleFilterLookupTabel/*.txt", lookup_files, false );
	particlefilter.load_lookup_table(lookup_files, map, 3, 8); 
	
	//particlefilter.generate_lookup_table(map, MeterPrPixel, Max_Meter, 1, 12); 

	float std[] = {0.1, 0.1, 0.1}; 
	particlefilter.init(0,0, 0, std); 
	
	cv::Mat particles = map.clone();
	particlefilter.draw_particles(particles, 0.1); 
	particlefilter.draw_robot_pos(particles, pos_rob_x, pos_rob_y, 0.1);

	cv::imwrite("../test/localization/particles.png", particles); 

	const int key_left = 81;
	const int key_up = 82;
	const int key_down = 84;
	const int key_right = 83;
	const int key_esc = 27;
	const int key_stop = 115;


	float speed = 0.0;
	float dir = 0.0;

	clock_t time = clock() ;

	double std2[] = {0.1, 0.1, 0.1};

	std::tuple<double,double,double> estimat;

	std::vector<std::tuple<double, double>> estimats;
	std::vector<std::tuple<double, double>> real;

	cv::VideoWriter outputVideo;  // Open the output
	outputVideo.open("../test/localization/movie.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, map.size()  , true);  //30 for 30 fps


	std::cout << "Entering endless loop" << std::endl; 

	std::ofstream position;
	position.open("../test/localization/position.txt");
	position << " \t \t Real \t\t estimat" << std::endl;
        position << " x \t y \t x \t y" << std::endl; 	

	while (true) {

		gazebo::common::Time::MSleep(50);     //
		mutex.lock();
		int key = cv::waitKey(1);
		mutex.unlock();

		// Localization
		clock_t newTime = clock(); 

		particlefilter.prediction( ((double) (newTime - time) ) / ((double)CLOCKS_PER_SEC) , std2 ,speed * 1, dir * 1  );
		while(flag); 
		flag = true;
		try { 
			estimat = particlefilter.dataAssociation(data, sigmaData, MeterPrPixel); 
		} catch(Empty e) {
			std::cout << "Error " << e.get_function()  << " " << e.get_line_number() << " " << e.get_info()  << std::endl;
			break;
		} 	
		flag = false;
		time = newTime;
		//

		{ 
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
			else if(key == key_stop) {
				speed = 0;
				dir = 0;

			}  
			//else {
			// slow down
			//      speed *= 0.1;
			//      dir *= 0.1;
			//}


			// Generate a pose
			ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

			// Convert to a pose message
			gazebo::msgs::Pose msg;
			gazebo::msgs::Set(&msg, pose);
			movementPublisher->Publish(msg);
		}

		{ 
			particles = map.clone();
			particlefilter.draw_particles(particles, 0.1); 
			particlefilter.draw_robot_pos(particles, pos_rob_x, -pos_rob_y, 0.1);
			particlefilter.draw_estime_pos(particles, std::get<0>(estimat), std::get<1>(estimat), 0.1); 


			estimats.push_back(std::tuple<double,double>(std::get<0>(estimat), std::get<1>(estimat)));
			real.push_back(std::tuple<double,double>(pos_rob_x, pos_rob_y));
		}

		{ 
			position << pos_rob_x  << " \t " << pos_rob_y << " \t " << std::get<0>(estimat) << " \t " << std::get<1>(estimat) << std::endl;    
			outputVideo << particles;
		}

		cv::imshow("Particles", particles); 
	}

	std::cout << "Drawing path" << std::endl; 

	position.close(); 


	cv::Mat dst;
	draw_rute(map,dst, real, cv::Scalar(255,0,0), MeterPrPixel, 2 , map_offset_x, map_offset_y); 
	draw_rute(dst,dst, estimats, cv::Scalar(0,0,255), MeterPrPixel, 2,  map_offset_x, map_offset_y); 

	cv::imwrite("../test/localization/path.png", dst); 


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
	float speed = 0.0;
	float dir = 0.0;
#endif

	controller.setGoal(NULL,NULL);
	//controller.setGoal(0/0 ,0/0);


	float marbleDir, marbleDist;
	bool marbleFound;
	bool collectMode = false;
	bool collectDone;
	const int marbleDetectionLimit = 3;
	int marbleDetections = 0;
	float mX, mY, mXnew, mYnew;
	float avgMarbleX = 0, avgMarbleY = 0;

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

		//        if ( marbleFound ) {
		//            if (marbleDetections < marbleDetectionLimit) {
		//                marbleDetections++;
		//                avgMarbleDir += marbleDir;
		//                avgMarbleDist += marbleDist;
		//                std::cout << "Marble detections: " << marbleDetections << std::endl;
		//                std::cout << "MarbleDir:    " << marbleDir << "\tMarbleDist:    " << marbleDist << std::endl;
		//            }
		//            if (marbleDetections == marbleDetectionLimit) {
		//                marbleDetections++;
		//                avgMarbleDir /= float(marbleDetectionLimit);
		//                avgMarbleDist /= float(marbleDetectionLimit);
		//                std::cout << "avgMarbleDir: " << avgMarbleDir << "\tavgMarbleDist: " << avgMarbleDist << std::endl;
		//
		//                controller.setMarble(avgMarbleDir, avgMarbleDist);
		//                collectMode = true; std::cout << "collectMode == true" << std::endl;
		//            }
		if ( marbleFound ) {
			if (marbleDetections < marbleDetectionLimit)
			{
				std::tie(mXnew, mYnew) = controller.globMarble(marbleDir, marbleDist);
				if (marbleDetections == 0) {
					mX = mXnew;
					mY = mYnew;
				}
				if (abs(mXnew - mX) < 0.5 && abs(mYnew - mY) < 0.5)
				{
					marbleDetections++;
					avgMarbleX += mXnew;
					avgMarbleY += mYnew;
					mX = mXnew;
					mY = mYnew;
					std::cout << "Marble detections: " << marbleDetections << std::endl;
					std::cout << "MarbleDir:    " << marbleDir << "\tMarbleDist:    " << marbleDist << std::endl;
					std::cout << "MarbleX:    " << mX << "\tMarbleY:    " << mY << std::endl;

				}
				else {
					marbleDetections = 0;
					avgMarbleX = 0;
					avgMarbleY = 0;
					mX = mXnew;
					mY = mYnew;
				}
			}



			if (marbleDetections == marbleDetectionLimit) {
				marbleDetections++;
				avgMarbleX /= float(marbleDetectionLimit);
				avgMarbleY /= float(marbleDetectionLimit);
				std::cout << "avgMarbleX: " << avgMarbleX << "\tavgMarbleY: " << avgMarbleY << std::endl;

				controller.setMarble(avgMarbleX, avgMarbleY);
				collectMode = true; std::cout << "collectMode == true" << std::endl;
			}
		}

		if (collectMode) {
			collectDone = controller.collect(speed, dir);
			if (collectDone) {
				collectMode = false;
				std::cout << "collectMode == false" << std::endl;
				avgMarbleX = 0;
				avgMarbleY = 0;
				marbleDetections = 0;
			}
		} else {
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
