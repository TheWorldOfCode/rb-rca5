#pragma once

#include <vector>

#include "line_detect.hpp" 
#include "vec2.hpp" 
#include "featureExtractor.hpp" 

#include <opencv2/opencv.hpp>

#define DEBUG_LOCALIZATION 1
#define MAP_COLOR cv::Vec3b(0,0,0) 
namespace localization {

	struct location {
	
		int x; // column
		int y; // row

		float yaw; // -180 < yaw <= 180 

	
	}; 

	class localization {

		public:
			localization(cv::Mat Map, const location c, const double meterPrPixel = -1);
			vec2 find_position(const std::vector<line> & lines);
			void draw_last_position_and_direction(cv::Mat & dst); 
			
			bool compare_features(const feature::features_t & f1, const feature::features_t & f2, const  line & dividere , const double distance_threshold); 

			~localization();

		private:
			cv::Mat map;
			double meterPrPixel;
			location current;

			std::vector<feature::features_t> mapFeatures;
			
	}; 
} 
