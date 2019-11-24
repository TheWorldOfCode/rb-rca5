
#include <random>
#include <limits>


#include "../includes/line_detect.hpp"

#if DEBUG_LINE_DETECT == 1
#include "../includes/vec2.hpp" 
#include <stdlib.h> 
#endif
namespace line_detect { 

	void split_and_merge(const std::vector<std::tuple<double, double>> & data, std::vector<line> & lines) {

		line_detect::split_and_merge(data, 0, data.size()-1 ,lines, 0.137310); 
	
	}  


	void split_and_merge(const std::vector<std::tuple<double,double>> & data, const int left, const int right, std::vector<line> & lines, const double lineDistanceThreshold) {


		if(data.size() > 1 && left < right) {

			line l(data[left],data[right]); 

			int splitPosition = -1;
			double maxDistance = std::numeric_limits<double>::min();  



			for(int i = left + 1; i < right ; i++ )
			{
				double distance = l.distance_to_point(data[i]);   

				if (distance < 0)
				       distance *= -1;	

				if(distance > lineDistanceThreshold) {
					if(distance  > maxDistance) {  
						maxDistance = distance;
						splitPosition = i;
					}
				}  

			}

			if(splitPosition > -1) {

				split_and_merge(data, left, splitPosition, lines, lineDistanceThreshold); 
				split_and_merge(data, splitPosition + 1, right, lines, lineDistanceThreshold); 
			} else
				lines.push_back(l); 	

		} 




	}  

	void RANSAC(const std::vector<std::tuple<double,double>> & data, std::vector<line> & lines) {
	
	 	line_detect::RANSAC(data, lines, 0, 0, 0); 
	} 

	void RANSAC(const std::vector<std::tuple<double,double>> & data, std::vector<line> & lines, const double threshold, const int inliers, const int iterations) {
	
		std::vector<std::tuple<double,double>> N  = data;

		std::default_random_engine generator;
		std::uniform_int_distribution<int> distribution(0, N.size()); 

		int iterator = 0;

		do { 

			std::vector<size_t> inlier; 

			size_t i = distribution(generator) % N.size();  
			size_t j = distribution(generator) % N.size();  

		
			line l(N[i], N[j]); 


			for(size_t x = 0; x < N.size(); x++ )
			{
				double distance =  l.distance_to_point(N[x]); 
				if(distance < 0)
				       distance *= -1;

			    if(distance < threshold )  
				    inlier.push_back(x); 
			}
			
			if(inlier.size() > (size_t) inliers ) {

				lines.push_back(l); 
			
				for(const size_t x : inlier  )
					N.erase(N.begin()+x); 
				
			}  

			inlier.clear(); 
		
			std::cout << "Remainder "  << N.size() << std::endl; 
			iterator++;
		} while((size_t) inliers < N.size() && iterator < iterations);

		N.clear(); 
	}  


#if DEBUG_LINE_DETECT > 0 

	line_detect::line_detect_test::line_detect_test(const int shots) : numberOfShots(shots) , flag(true)  {}  

	//line_detect_test::line_detect_test(std::vector<std::tuple<double, double>> & data) : data(data) {}   

	void line_detect::line_detect_test::wait_on_data() { while(flag) {} } 

	void line_detect::line_detect_test::draw_lines(const std::vector<line> & lines, cv::Mat & dst) {
		std::cout << lines.size() << std::endl; 

		int width = 400;
		int height = 400;
		float px_per_m = 200 / range_max;

		cv::Mat im(height, width, CV_8UC3);
		im.setTo(255); // Set background color
		vec2 startpt, endpt;
		for(const line l : lines )
		{
			l.get_point_to_draw_the_line(startpt, endpt); 	


			cv::Point2f startPt(startpt.getX()*px_per_m +200.5f, 200.5f - px_per_m * startpt.getY()  );
			cv::Point2f endPt(200.5f + px_per_m * endpt.getX(), 200.5f -px_per_m *  endpt.getY()  ) ;


			cv::line(im, startPt * 16, endPt * 16, cv::Scalar(0,0,0), 1, cv::LINE_AA, 4);
		   
		}

		dst = im.clone(); 
	}  

	void line_detect::line_detect_test::show_raw_lidar_information() {


		int width = 400;
		int height = 400;
		float px_per_m = 200 / range_max;

		cv::Mat im(height, width, CV_8UC3);
		im.setTo(0);
		for (size_t i = 0; i < data.size() ; i++) {
			double range = std::get<0>(data[i]); 
			double angle = std::get<1>(data[i]); 

			cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
					200.5f - range_min * px_per_m * std::sin(angle));
			cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
					200.5f - range * px_per_m * std::sin(angle));
			cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
					cv::LINE_AA, 4);
		}

		cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));

		cv::imshow("lidar raw information", im);
	}  

	void line_detect::line_detect_test::show_raw_lidar_information(cv::Mat &out) {


		int width = 400;
		int height = 400;
		px_per_m = 200 / range_max;

		cv::Mat im(height, width, CV_8UC3);
		im.setTo(0);
		for (size_t i = 0; i < data.size() ; i++) {
			double range = std::get<0>(data[i]); 
			double angle = std::get<1>(data[i]); 

			cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
					200.5f - range_min * px_per_m * std::sin(angle));
			cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
					200.5f - range * px_per_m * std::sin(angle));
			cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1, cv::LINE_AA, 4);

/*			if(45 < i && i < 82 ) { 
			       cv::circle(im, endpt, 2, cv::Scalar(255,255,0)); 	
			}*/
		}

		cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));

		out = im.clone(); 
	}  

	void line_detect::line_detect_test::show_lidar_information_with_lines(std::vector<line> lines, std::string filename ) {

		cv::Mat raw; 
		show_raw_lidar_information(raw);

		vec2 startpt, endpt;
		for(const line l : lines )
		{
			l.get_point_to_draw_the_line(startpt, endpt); 	


			cv::Point2f startPt(startpt.getX()*px_per_m +200.5f, 200.5f - px_per_m * startpt.getY()  );
			cv::Point2f endPt(200.5f + px_per_m * endpt.getX(), 200.5f -px_per_m *  endpt.getY()  ) ;


			cv::line(raw, startPt * 16, endPt * 16, cv::Scalar(rand() % 255, rand()  %  255, rand() %  255, rand() %  255), 2, cv::LINE_AA, 4);
		   
		}
		
		std::cout << filename.length() << std::endl; 
		if(filename.length() == 0 ) 
			cv::imshow("lidar with lines", raw);
		else
			cv::imwrite(filename.c_str() ,raw ); 


	}  

	void line_detect::line_detect_test::split_and_merge(const double lineDistanceThreshold, std::vector<line> & lines) {

		if(flag == false) {  
			std::vector<line> tlines;

			line_detect::split_and_merge(data, 0, data.size()-1, tlines, lineDistanceThreshold); 	
			show_lidar_information_with_lines(tlines, ""); 

			lines = tlines;

		}
	}

	void line_detect::line_detect_test::split_and_merge(const double lineDistanceThreshold) {

		if(flag == false) {  
			std::cout << "Test split and merge line detecting with line distance threshold: " << lineDistanceThreshold << std::endl; 
			std::vector<line> lines;

			line_detect::split_and_merge(data, 0, data.size()-1, lines, lineDistanceThreshold); 	

			std::cout << std::endl;

			std::cout << "There where detect " << lines.size() << " lines" << std::endl;   

			show_raw_lidar_information();
			show_lidar_information_with_lines(lines); 	
			cv::waitKey(); 

		}

	}
	
	void line_detect::line_detect_test::split_and_merge_show_steps(const double lineDistanceThreshold) {

		if(flag == false) {  
			imageNumber = 0;
			std::cout << std::endl;
			std::cout << "Test split and merge line detecting with line distance threshold: " << (double) lineDistanceThreshold << std::endl; 
			std::vector<line> lines;

			split_and_merge_test(data, 0, data.size()-1, lines, lineDistanceThreshold); 	


			std::cout << "There where detect " << lines.size() << " lines" << std::endl;   
			show_lidar_information_with_lines(lines, "../test/lidar/line/allLines_T" + std::to_string(lineDistanceThreshold)+ " _LS"+ std::to_string(lines.size())  +".png" ); 	

		}

	}

	void line_detect::line_detect_test::split_and_merge_test(const std::vector<std::tuple<double,double>> & data, const int left, const int right, std::vector<line> & lines, const double lineDistanceThreshold) {



		if(data.size() > 1 && left < right) {

			line l(data[left],data[right]); 

			int splitPosition = -1;
			double maxDistance = std::numeric_limits<double>::min()  ;


			for(int i = left + 1; i < right ; i++ )
			{
				double distance = l.distance_to_point(data[i]);   

				if(distance < 0)
				       distance *= -1;	

				if(distance > lineDistanceThreshold) {
					if(distance > maxDistance) { 
						splitPosition = i;
						maxDistance = distance;
					}
				}  

			}

			std::vector<line> test;
			test.push_back(l); 
			/*
			if(imageNumber < 10) 
				show_lidar_information_with_lines(test, "../test/lidar/line/00"+ std::to_string(imageNumber)    + "_L" + std::to_string(left) +"_S" + std::to_string(splitPosition) + "_R" + std::to_string(right) + "_DS" + std::to_string(data.size()) + "_LS" + std::to_string(lines.size()) + "_T" + std::to_string(lineDistanceThreshold) + "_BD" + std::to_string(maxDistance) +  ".png"  ); 
			else if(imageNumber < 100) 
				show_lidar_information_with_lines(test, "../test/lidar/line/0"+ std::to_string(imageNumber)    + "_L" + std::to_string(left) +"_S" + std::to_string(splitPosition) + "_R" + std::to_string(right) + "_DS" + std::to_string(data.size()) + "_LS" + std::to_string(lines.size()) + "_T" + std::to_string(lineDistanceThreshold) + "_BD"  + std::to_string(maxDistance) + ".png"  ); 
			else 
				show_lidar_information_with_lines(test, "../test/lidar/line/"+ std::to_string(imageNumber)    + "_L" + std::to_string(left) +"_S" + std::to_string(splitPosition) + "_R" + std::to_string(right) + "_DS" + std::to_string(data.size()) + "_LS" + std::to_string(lines.size()) + "_T" + std::to_string(lineDistanceThreshold) + "_BD"  + std::to_string(maxDistance)  + ".png"  ); 

*/
			imageNumber++;
			
			if(splitPosition > -1) {

				split_and_merge_test(data, left, splitPosition, lines, lineDistanceThreshold); 
				split_and_merge_test(data, splitPosition + 1, right, lines, lineDistanceThreshold); 
			} else  
				lines.push_back(l); 	

		} 




	}  


	void line_detect::line_detect_test::RANSAC(const double threshold, const int inlier, const int iterations, std::vector<line> & lines) {

		if(flag == false) {  
			std::cout << "Test RANSAC line detecting with line distance threshold: " << threshold << " Number of inliers " << inlier << " Number of max iterations " << iterations   << std::endl; 
			std::vector<line> lines;

			line_detect::RANSAC(data, lines, threshold, inlier, iterations); 

			std::cout << std::endl;

			std::cout << "There where detect " << lines.size() << " lines" << std::endl;   

			show_raw_lidar_information();
			show_lidar_information_with_lines(lines); 	
			cv::waitKey(); 

		}

	}  	


	void line_detect::line_detect_test::lidarCallback(ConstLaserScanStampedPtr &msg) {


		float angle_min = float(msg->scan().angle_min());
		float angle_increment = float(msg->scan().angle_step());

		range_min = float(msg->scan().range_min());
		range_max = float(msg->scan().range_max());

		int nranges = msg->scan().ranges_size();
		int nintensities = msg->scan().intensities_size();

		assert(nranges == nintensities);

		if(flag == true && numberOfShots > 0) {  	
			numberOfShots--;

			flag = false;
			data.clear(); 

			for (int i = 0; i < nranges; i++) {
				float angle = angle_min + i * angle_increment;
				float range = std::min(float(msg->scan().ranges(i)), range_max);

				data.push_back(std::tuple<double, double>(range, angle));
			}
		}

	}

	line_detect::line_detect_test::~line_detect_test() {} 
#endif
}
