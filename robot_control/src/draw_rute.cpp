#include "../includes/draw_rute.hpp" 


void draw_rute(const cv::Mat & src, cv::Mat & dst, const std::vector<double> x, const std::vector<double> y, const cv::Scalar color, const int size, const int offset_x, const int offset_y)  {

	dst = src.clone(); 


	if(x.size() != y.size())
	       throw SizeError("The vectors x and y has not the same size", "draw_rute",10); 	


	for(size_t i = 0; i < x.size(); i++ )
	{
	     cv::Point pt(x+offset_x, y+offset_y);  

	     cv::circle(dst, pt, size, color, -1,8,0); 
	}
	
	

} 

void draw_rute(const cv::Mat & src, cv::Mat & dst, const std::vector<std::tuple<double,  double, double>> data, const cv::Scalar color, const int size, const int offset_x , const int offset_y) {

	std::vector<double> x;
	std::vector<double> y;
	for(const std::tuple<double, double, double>  d :  data )
	{
	       x.push_back(std::get<0>(d));   
	       y.push_back(std::get<1>(d));   
	}
	
	
	draw_rute(src,dst, x, y, size, color, offset_x, offset_y); 


} 

