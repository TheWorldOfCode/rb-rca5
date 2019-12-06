#pragma once

#include <vector>

#include <opencv2/opencv.hpp>
class SizeError : std::exception {

	public: 
		Empty(std::string info, std::string function, int line_number) : info(info), function(function), line_number(line_number) {}  
		std::string get_info() const noexcept{ return info; }  
		std::string get_function() const noexcept { return function; } 
	        int get_line_number() const noexcept { return line_number; }  	
		~Empty() {} 
	private: 
		std::string info;
		std::string function;
		int line_number;


}; 



void draw_rute(const cv::Mat & src, cv::Mat & dst, const std::vector<double> x, const std::vector<double> y, const cv::Scalar color, const int size = 1, const int offset_x = 0, const int offset_y = 0 ); 

// data is x y theta. Theta is not used
void draw_rute(const cv::Mat & src, cv::Mat & dst, const std::vector<std::tuple<double,  double, double>> data, const cv::Scalar color, const int size = 1, const int offset_x = 0, const int offset_y = 0 ); 
