#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write
#include <iostream>
#include <vector>
#include <tuple>
#include <unistd.h> 
#include <time.h>


using namespace std;

int main() {

	cv::Mat map = cv::imread("../map/bigworld_floor_plan.png");  

	const int cols = map.cols;
	const int rows = map.rows;
	std::cout << cols << std::endl;


	cv::resize(map, map, cv::Size(), 1/1.41735 * 10, 1/1.41735 * 10); 
	cv::imwrite("../test/localization/orignalMap.png", map );

	const double resolutation_angle = 1;
	const double MeterPrPixel = 0.1;
	const double max_meter = 10;

	int i,j;
	std::cout << map.rows/2 << " " << map.cols/2 << std::endl; 

	std::cout << "Select col" << std::endl;
        cin >> i;
	std::cout << "Select row" 	<< std::endl;
	cin >> j;



	vector<cv::Point> k;
	vector<double> angle2;
	vector<cv::Point> hit;
	for(float angle = 0; angle < 360; angle+=resolutation_angle  )
	{

		cv::Point p1(i,j);  
		cv::Point p2(i + cos(angle * M_PI/((double) 180 ) ) * max_meter/MeterPrPixel * 1.2, j - sin(angle* M_PI/((double) 180 ) ) * max_meter/MeterPrPixel * 1.2 );

		cv::LineIterator itr(map, p1, p2 ); 
		int count = 0;
		
		while(map.at<cv::Vec3b>(itr.pos()) == cv::Vec3b(255,255,255) && itr.count != count  ) { 
			const cv::Point tmp(itr.pos().x, itr.pos().y) ; 
			const int x = tmp.x;
			const int y = tmp.y;

			if(cols - x > 0 && x < 0 ) 
				break;

			if(y < 0 && rows-y > 0) 
				break;

			++count;
			++itr;
		}

		
		const cv::Point tmp(itr.pos().x, itr.pos().y) ; 
		double meter = min(sqrt( (tmp.x - p1.x) * (tmp.x - p1.x) + (tmp.y -p1.y) * (tmp.y -p1.y) )* MeterPrPixel, max_meter);

		std::cout << "Position A " <<  p1 << " Position B " << p2 <<  " Angle: " << angle << " itr count " <<   itr.count << " Moved Before black " << count << " Distance in meter " << meter << std::endl;  
		std::cout << sqrt( (tmp.x - p1.x) * (tmp.x - p1.x) + (tmp.y -p1.y) * (tmp.y -p1.y) ) * MeterPrPixel  << std::endl;


		angle2.push_back(angle); 
		k.push_back(p2); 
		hit.push_back(itr.pos()); 


	}


	cv::circle(map, cv::Point(i,j) , 4, cv::Vec3b(255,0,0)); 
	for(size_t i = 0; i < angle2.size(); i++  )
	{
		cv::Point pt = k[i];
		cv::Point pt2 = hit[i];

		if(angle2[i] == 90)  { 
			cv::circle(map, pt, 4, cv::Vec3b(0,0,255)); 
			cv::circle(map, pt2, 4, cv::Vec3b(255,0,255)); 
		} else { 
			cv::circle(map, pt, 4, cv::Vec3b(0,255,0)); 
			cv::circle(map, pt2, 4, cv::Vec3b(255,255,0)); 
		}
	}



	cv::imshow("Map", map);

	cv::waitKey(); 	

	return 0;
}
