
#include <iostream>
#include <cmath>
#include <limits>

#include "../includes/localization.hpp"
#include "../includes/featureExtractor.hpp" 

#define PIXEL_TO_METER(x) x/72.0 * 25.4 
// mm = pt/72 * 25.4

#define DISTANCE_THRESHOLD 0.1 

using namespace feature;
namespace localization {

	localization::localization(cv::Mat Map, const location c, const double meterPrPixel ) :  meterPrPixel(meterPrPixel), current(c)   {
		map = Map;

#if DEBUG_LOCALIZATION == 1 
		featureExtractorCornor(map, map, mapFeatures, MAP_COLOR ); 

		std::cout << "Number of features " << mapFeatures.size() << std::endl;   
		for(size_t i = 0;  i < mapFeatures.size() ; i++ )
		{
			std::cout << "Current feature " << i << " "  << mapFeatures[i]  << std::endl;
		}
		
		
	
#else

		featureExtractorCornor(map, mapFeatures, MAP_COLOR ); 
#endif

	}

	bool compare_features(const features_t & f1, const features_t & f2, const line & dividere , const double distance_threshold) {

		
		if(f1.type != f2.type)
			return false;

		std::cout << "Same type" << std::endl;
		double distance = dividere.distance_to_point(vec2(f2.x, f2.y));
		double absDistance = std::abs(distance); 

		
		if (distance != 0 &&  distance/absDistance  != f1.y/std::abs(f1.y)) 
			return false;

		std::cout << "On the same side of dividere" << std::endl; 
		double length = vec2(f1.x,f1.y).length();   

		std::cout << distance << " " << length << std::endl; 
		if(absDistance - distance_threshold < length && length < absDistance + distance_threshold) 
			return true;


		return false;
		
	}

	void compare_features_interface(line dividere, std::vector<features_t> mapFeatures ,std::vector<features_t> features) {
	

		std::cout << "Dividing line " << dividere << std::endl;  
		int a = 0;
		int b = 0;
		double threshold;
		while(a != -1) {
		
			std::cout <<"Select map feature to compare (" << mapFeatures.size()  << ")" << std::endl;    
			std::cin >> a;

			std::cout <<"Select line feature to compare (" << features.size()  << ")" << std::endl;    
			std::cin >> b;

			std::cout << "Select threshold" << std::endl;
			std::cin >> threshold; 	

			
			if(compare_features(features[b], mapFeatures[a], dividere, threshold) ) 
				std::cout << "The feature are the \"same\" "  << "true"  << std::endl; 
			else 
				std::cout << "The feature are the \"same\" "  << "false"  << std::endl; 
			std::cout << "Map feature " << mapFeatures[a] << std::endl; 
			std::cout << "feature " << features[b] << std::endl; 

		
		
		}  

	
	}  


	vec2 localization::find_position(const std::vector<line> & lines) {

#if DEBUG_LOCALIZATION == 1
		cv::imshow("floor plan", map ); 
#endif 	
		
		std::vector<features_t> features; 
		featureExtractorCornor(lines, features, 0.5); 

#if DEBUG_LOCALIZATION == 1

		std::cout << "Number of features " << features.size() << " from lines"  << std::endl;   
		for(size_t i = 0;  i < features.size() ; i++ )
		{
			std::cout << "Current feature " << i << " "  << features[i]  << std::endl;
		}
#endif

/*		for(size_t i = 0; i < features.size(); i++ )
		{
		   if(features[i].numberOfConnected == 0 )
			  features.erase(features.begin() + 1 );   
		}
		*/
#if DEBUG_LOCALIZATION == 1
		cv::Mat position = map.clone() ;
		draw_last_position_and_direction(position); 
		cv::imshow("P and D", position ); 
#endif 
		
		vec2 start(current.x, current.y);
	        vec2 stop(current.x + std::cos(current.yaw) * 2, current.y - std::sin(current.yaw) * 2);

		line dividere(start, stop, pointType::CARTESIAN); 

#if DEBUG_LOCALIZATION == 1	
		std::cout << "Current estime " <<  current.x << " " << current.y << std::endl;  
		std::cout << "Dividere "  <<  dividere << std::endl;

		features_t tmp = mapFeatures[mapFeatures.size()-3];
	        std::cout << tmp << std::endl;	

		std::cout << vec2(features[0].x, features[1].y).length() << std::endl;  
		std::cout << dividere.distance_to_point(vec2(tmp.x, tmp.y))/72 * 25.4 << std::endl;
//		compare_features_interface(dividere, mapFeatures, features); 
#endif 

		std::vector<std::tuple<features_t, features_t>> connected;

		int tmp1 = 0;
		int tmp2 = 0;
	
		for(features_t f2 : mapFeatures )
		{
		   	for(features_t f1 : features )
		   	{
				if(compare_features(f1,f2,dividere, DISTANCE_THRESHOLD)) {
				 
					 for(int i = 0; i < f1.numberOfConnected; i++ ) { 
					 
						 for(int j = 0; j < f2.numberOfConnected; j++ ) {


							 if( compare_features(features[f1.connected[i]], mapFeatures[f2.connected[j]], dividere, DISTANCE_THRESHOLD)) { 
								 connected.push_back(std::tuple<features_t, features_t>(f1,f2)); 
								 break;
							 }
						 }
					 }

					 
				 } 
			}


		}

#if DEBUG_LOCALIZATION == 1
		std::cout << "Found  "  << connected.size() << std::endl; 

		std::cout << std::endl;
		cv::Mat select_features = position.clone() ;
		for( const std::tuple<features_t, features_t> t : connected )
		{
		   features_t f = std::get<1>(t);
		   features_t f0 = std::get<0>(t);

		   std::cout << "Map " << f << std::endl;
		   std::cout << "Line " << f0 << std::endl;   
		   std::cout << std::endl;
		   select_features.at<cv::Vec3b>(f.y, f.x) = cv::Vec3b(255,255,0);  
		}
		
		
#endif

	// Filtering 
	for(size_t i = 0; i < connected.size(); i++  )
	{
		std::tuple<features_t, features_t> t = connected[i];
		features_t f11 = std::get<0>(t);
	        features_t f12 = std::get<1>(t); 	

		for(size_t j = 0; j < connected.size(); j++) 
		{
			std::tuple<features_t, features_t> t2 = connected[j];

			features_t f21 = std::get<0>(t2); 
			features_t f22 = std::get<1>(t2); 

			if(f11.index == f21.index) {
				
			
				vec2 r1 = vec2(f12.x, f12.y); 
				vec2 r2 = vec2(f22.x, f22.y); 

				double distanceR1 = ((vec2) (r1-vec2(current.x, current.y))).length(); 
				double distanceR2 = ((vec2) (r2-vec2(current.x, current.y))).length(); 
			
				if(distanceR1 < distanceR2) 
					connected.erase(connected.begin() + j ); 
				else if (distanceR2 < distanceR1) {
					connected.erase(connected.begin() + i ); 
					break;
				
				}  
				
			
			}  
		   
		}
		
		
	   
	}
	
	

#if DEBUG_LOCALIZATION == 1
		std::cout << "Found  "  << connected.size() << std::endl; 

		std::cout << std::endl;
		cv::Mat Estime = position.clone() ;
		for( const std::tuple<features_t, features_t> t : connected )
		{
		   features_t f = std::get<1>(t);
		   features_t f0 = std::get<0>(t);

		   std::cout << "Map " << f << std::endl;
		   std::cout << "Line " << f0 << std::endl;   
		   std::cout << std::endl;
		   Estime.at<cv::Vec3b>(f.y, f.x) = cv::Vec3b(255,255,0);  
		}
		
		
#endif




#if DEBUG_LOCALIZATION == 1
		cv::waitKey();
		cv::imwrite("../test/mapFeatures.png", map); 
		cv::imwrite("../test/mapPosition.png", position ); 
		cv::imwrite("../test/mapSelectedFeatures.png", select_features ); 
		cv::imwrite("../test/mapEstime.png", Estime ); 
#endif	
		return vec2(0,0);  

	}  

	void localization::draw_last_position_and_direction(cv::Mat & dst)  {

		cv::Point start(current.x, current.y); 
		cv::Point end(current.x+ std::cos(current.yaw) * 6,current.y - std::sin(current.yaw) * 6 );

		cv::line(dst, start,end, cv::Vec3b(0,255,255)); 	

		dst.at<cv::Vec3b>(current.y, current.x) = cv::Vec3b(0,0,255);   
	} 

          
        bool localization::compare_features(const features_t & f1, const features_t & f2, const line & dividere , const double distance_threshold) {

		
		if(f1.type != f2.type)
			return false;

		double distance = PIXEL_TO_METER(dividere.distance_to_point(vec2(f2.x, f2.y)));
		double absDistance = std::abs(distance); 

		if (distance != 0 && distance/absDistance  != f1.y/std::abs(f1.y)) 
			return false;

		double length = vec2(f1.x,f1.y).length();   

		if(absDistance - distance_threshold < length && length < absDistance + distance_threshold) 
			return true;


		return false;
		
	}  

	localization::~localization() {}  
}

