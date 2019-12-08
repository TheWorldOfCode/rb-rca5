#include "../includes/brushfire.hpp"
#include "../includes/roadmap.hpp"


#include <opencv2/opencv.hpp>
#include <iostream>

void takepart(cv::Mat & src, cv::Mat & dst, const int row_begin, const int row_end, const int col_begin, const int col_end) {

	dst = cv::Mat(row_end - row_begin, col_end-col_begin, src.type()); 
	

	for(int row = row_begin;  row < row_end; row++ )
		for(int col = col_begin; col < col_end; col++ )
			dst.at<cv::Vec3b>(row - row_begin, col - col_begin) = src.at<cv::Vec3b>(row, col);    


}  


int main(int _argc, char **_argv) {
	std::cout << "Loading big map" << std::endl; 
	cv::Mat map = cv::imread("../map/bigworld_floor_plan.png"); 

	std::cout << "Creating brushfire" << std::endl;

	cv::Mat brushfire;
	int max = generate_brushfire(map, brushfire); 

	cv::imshow("brushfire", brushfire);
	cv::imwrite("brushfire.png", brushfire); 	

	std::cout << "Create roadmap" << std::endl;

	roadmap roadMap(brushfire, max); 

	std::cout << "Drawing roadmap" << std::endl;

	cv::Mat roads;

	roadMap.draw_roadmap(roads); 

	cv::Mat roadsOverlay = roads.clone(); 
	roadMap.draw_world_overlay(roadsOverlay, map);  


	cv::imshow("Roadmap", roads);
	cv::imwrite("Roadmap.png", roads); 
	cv::imshow("Roadmap with walls", roadsOverlay); 	
	cv::imwrite("RoadmapWithWalls.png", roadsOverlay); 
	cv::Mat error;
	takepart(roadsOverlay, error, 0, roadsOverlay.rows - 40,roadsOverlay.cols-30, roadsOverlay.cols); 

	cv::imwrite("roadmapPart3Error.png", error); 


	std::cout << "Done" << std::endl;
	cv::waitKey(); 	

	return 0;
}
