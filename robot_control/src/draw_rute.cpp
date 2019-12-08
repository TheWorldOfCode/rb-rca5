#include "../includes/draw_rute.hpp" 


void draw_rute(const cv::Mat & src, cv::Mat & dst, const std::vector<double> x, const std::vector<double> y, const cv::Scalar color, const int size, const int offset_x, const int offset_y)  {

	dst = src.clone(); 


	if(x.size() != y.size())
	       throw SizeError("The vectors x and y has not the same size", "draw_rute",10); 	


	for(size_t i = 0; i < x.size(); i++ )
	{
	     cv::Point pt(x[i]+offset_x, y[i]+offset_y);

	     cv::circle(dst, pt, size, color, -1,8,0); 
	}
	
	

} 

void draw_rute(const cv::Mat & src, cv::Mat & dst, const std::vector<std::tuple<double,  double>> data, const cv::Scalar color, const double MeterPrPixel, const int size, const int offset_x , const int offset_y) {

	std::vector<double> x;
	std::vector<double> y;
	for(const std::tuple<double, double>  d :  data )
	{
	       x.push_back(std::round(std::get<0>(d) / MeterPrPixel));   
	       y.push_back(-1 * std::round(std::get<1>(d) / MeterPrPixel));   
	}
	
	draw_rute(src,dst, x, y, color, size, offset_x, offset_y);


} 

