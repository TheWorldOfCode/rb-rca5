#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

#include "line.hpp" 

namespace feature {

	enum featureType {NaN,INDRE, OUTER};
	enum featureLocalation {NoN, LEFT_APOVE, LEFT_BELOW, RIGHT_APOVE, RIGHT_BELOW};

	struct features_t {
		double x;
		double y;
		featureType type = featureType::NaN;
		featureLocalation loc = featureLocalation::NoN;
		int index = -1;
		int numberOfConnected = 0; 
		int connected[2];

	}; 

	std::ostream & operator<<(std::ostream & os, features_t & f); 

        void featureExtractorCornor(const cv::Mat &  map, std::vector<features_t> & f, cv::Vec3b mapColor ); 
        void featureExtractorCornor(const cv::Mat &  map, cv::Mat & dst, std::vector<features_t> & f, cv::Vec3b mapColor ); 



        void featureExtractorCornor(const std::vector<line> & lines, std::vector<features_t> & f, const double threshold ); 


} 
