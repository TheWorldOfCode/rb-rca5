#pragma once

#include <opencv2/opencv.hpp>

#define BRUSHFIRE_STEP_SIZE 10
#define BRUSHFIRE_BEGIN 100

#define DEBUG_BRUSHFIRE  0


int generate_brushfire(cv::Mat & source, cv::Mat & dst); 
