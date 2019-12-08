#pragma once

#include <tuple>
#include <vector>

#include "line.hpp" 

#define DEBUG_LINE_DETECT 0
#define DEBUG_LINE_DETECT_VECTOR_TEST 0 
#define DEBUG_LINE_DETECT_SPLIT_AND_MERGE 0
#define DEBUG_LINE_DETECT_SPLIT_AND_MERGE_FIND_THRESHOLD 0
#define DEBUG_LINE_DETECT_RANSAC 0

#if DEBUG_LINE_DETECT > 0

#include <gazebo/msgs/msgs.hh>
#include <opencv2/opencv.hpp>
#include <string>

#endif

namespace line_detect {

	void split_and_merge(const std::vector<std::tuple<double, double>> & data, std::vector<line> & lines); 
	void split_and_merge(const std::vector<std::tuple<double,double>> & data, const int left, const int right, std::vector<line> & lines, const double listDistanceThreshold); 

	void RANSAC(const std::vector<std::tuple<double,double>> & data, std::vector<line> & lines);
	void RANSAC(const std::vector<std::tuple<double,double>> & data, std::vector<line> & lines, const double threshold, const int inliers, const int iterations);



#if DEBUG_LINE_DETECT > 0 
	class line_detect_test {
	
		public: 
			line_detect_test(const int shots); 
//			line_detect_test(std::vector<std::tuple<double,double>> & data);
			void wait_on_data(); 


			void draw_lines(const std::vector<line> & lines, cv::Mat & dst); 
			void show_raw_lidar_information(); 
			void show_raw_lidar_information(cv::Mat &out); 
			void show_lidar_information_with_lines(std::vector<line> lines, std::string filename = "" ); 

			void split_and_merge(const double lineDistanceThreshold, std::vector<line> & lines); 
			void split_and_merge(const double lineDistanceThreshold); 
			void split_and_merge_show_steps(const double lineDistanceThreshold); 
			void split_and_merge_test(const std::vector<std::tuple<double,double>> & data, const int left, const int right, std::vector<line> & lines, const double lineDistanceThreshold); 

			void RANSAC(const double threshold, const int inliers, const int iterations, std::vector<line> & lines); 

			void lidarCallback(ConstLaserScanStampedPtr &msg); 

			~line_detect_test(); 
		private:
			std::vector<std::tuple<double, double>> data;	
			float range_max;
			float range_min;
			float px_per_m;
			int numberOfShots;
			int imageNumber;
	

			bool flag;
	
	}; 
#endif
} 
