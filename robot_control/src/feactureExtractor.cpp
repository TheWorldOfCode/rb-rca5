
#include "../includes/featureExtractor.hpp" 

#define PIXEL_APOVE(row, col) map.at<cv::Vec3b>(row - 1, col) == mapColor  
#define PIXEL_LEFT(row, col)  map.at<cv::Vec3b>(row, col - 1) == mapColor 
#define PIXEL_RIGHT(row, col) map.at<cv::Vec3b>(row, col + 1) == mapColor   
#define PIXEL_DOWN(row, col)  map.at<cv::Vec3b>(row + 1, col) == mapColor

#define PIXEL_APOVE_RIGHT(row, col) map.at<cv::Vec3b>(row - 1, col + 1) == mapColor   
#define PIXEL_APOVE_LEFT(row, col) map.at<cv::Vec3b>(row - 1, col - 1) == mapColor   
#define PIXEL_DOWN_RIGHT(row, col) map.at<cv::Vec3b>(row + 1, col + 1) == mapColor   
#define PIXEL_DOWN_LEFT(row, col) map.at<cv::Vec3b>(row + 1, col - 1) == mapColor   

#define CHECK_LEFT_APOVE(x) f[x].loc == featureLocalation::LEFT_APOVE
#define CHECK_RIGHT_APOVE(x) f[x].loc == featureLocalation::RIGHT_APOVE 
#define CHECK_LEFT_BELOW(x) f[x].loc == featureLocalation::LEFT_BELOW
#define CHECK_RIGHT_BELOW(x) f[x].loc == featureLocalation::RIGHT_BELOW 
namespace feature {

	std::ostream & operator<<(std::ostream & os, features_t & f) {

		os << "Position(x = col, y = row) : ("<< f.x << "," << f.y << " )  type: " ;		       
		switch(f.type) {

			case NaN:
				os << "NaN";
				break;
			case INDRE: 	
				os << "INDRE";
				break;
			case OUTER: 
				os << "OUTER"; 
				break;

		}  
		os << " loc: " ;

		switch (f.loc) {

			case NoN: 
				os << "NaN";
				break;
			case LEFT_APOVE:
				os << "LEFT APOVE";	
				break;
			case LEFT_BELOW:
				os << "LEFT BELOW";
				break;
			case RIGHT_APOVE:
				os << "RIGHT APOVE";
				break;
			case RIGHT_BELOW:
				os << "RIGHT BELOW";
				break;

		}  
		os <<	" Index (line)  " << f.index <<   " number of connected features " << f.numberOfConnected  << " Index of connected features ["; 

		if(f.numberOfConnected == 1)
			os << f.connected[0];  	
		else if(f.numberOfConnected == 2) 
			os << f.connected[0] << " " << f.connected[1];  	

		os << "]"; 

		return os;

	} 


	void inline find_connections_row(std::vector<features_t> & f, const int & i, const int & j) {


		if(f[i].type == featureType::INDRE && f[j].type == featureType::INDRE) {

			if( f[i].loc != f[j].loc) {
				f[i].connected[f[i].numberOfConnected++] = j;
				f[j].connected[f[j].numberOfConnected++] = i;
			}  

		} 

	}  


	void inline find_connections_row_dif(std::vector<features_t> & f, const int & i, const int & j) {

		if(f[i].y < f[j].y) {

			if ((CHECK_LEFT_APOVE(i) && CHECK_LEFT_APOVE(j)) || (CHECK_RIGHT_APOVE(i) && CHECK_RIGHT_APOVE(j))) {
				f[i].connected[f[i].numberOfConnected++] = j;
				f[j].connected[f[j].numberOfConnected++] = i;
			}  

		} else if (f[j].y < f[i].y) {
			if ((CHECK_LEFT_BELOW(i) && CHECK_LEFT_BELOW(j) ) || (CHECK_RIGHT_BELOW(i) && CHECK_RIGHT_BELOW(j) )  ) {

				f[i].connected[f[i].numberOfConnected++] = j;
				f[j].connected[f[j].numberOfConnected++] = i;
			} 

		}   


	}  

	void inline find_connections_col_out(std::vector<features_t> & f, const int & i, const int & j ) {

		if(f[i].y < f[j].y) {

			if ((CHECK_RIGHT_BELOW(i) && CHECK_RIGHT_APOVE(j) ) || (CHECK_LEFT_BELOW(i) && CHECK_LEFT_APOVE(j))) {
				f[i].connected[f[i].numberOfConnected++] = j;
				f[j].connected[f[j].numberOfConnected++] = i;
			}  

		} else 
			find_connections_col_out(f,j,i); 	
	}  

	void inline find_connections_col(std::vector<features_t> & f, const int & i, const int & j) {

		if(f[i].type == featureType::INDRE && f[j].type == featureType::INDRE) {

			if(f[i].y < f[j].y) {

				if( (CHECK_LEFT_APOVE(i) && CHECK_LEFT_BELOW(j)) || (CHECK_RIGHT_APOVE(i) && CHECK_RIGHT_BELOW(j)) ) {

					f[i].connected[f[i].numberOfConnected++] = j;
					f[j].connected[f[j].numberOfConnected++] = i;

				} 

			} else if (f[j].y < f[i].y) {

				if( (CHECK_LEFT_APOVE(j) && CHECK_LEFT_BELOW(i)) || (CHECK_RIGHT_APOVE(j) && CHECK_RIGHT_BELOW(i)) ) {

					f[i].connected[f[i].numberOfConnected++] = j;
					f[j].connected[f[j].numberOfConnected++] = i;

				} 
			}  

		} else if (f[i].type == featureType::INDRE && f[j].type == featureType::OUTER) 
			find_connections_row_dif(f, i,j);
		else if (f[j].type == featureType::INDRE && f[i].type == featureType::OUTER) 
			find_connections_row_dif(f,j,i); 
		else 
			find_connections_col_out(f,i,j); 


	}  

	void find_connections(std::vector<features_t> & f) {

		for(size_t i = 0; i < f.size(); i++ )
		{
			for(size_t j = 0; j < f.size(); j++ )
			{
				if( i != j && f[i].numberOfConnected < 2 && f[j].numberOfConnected < 2) {

					//					if(f[i].y == f[j].y ) 
					// 						find_connections_row(f, i, j); 
					//					else if (f[i].x == f[j].x)  
					if(f[i].x == f[j].x) 
						find_connections_col(f,i,j); 
				}  
			}
		}
	}  



	void featureExtractorCornor(const cv::Mat & map, std::vector<features_t> & f, cv::Vec3b mapColor) {

		int rows = map.rows;
		int cols = map.cols;

		for(int row = 1; row < rows -1 ;  row++)
		{
			for(int col = 1; col < cols -1 ; col++ )
			{

				if(map.at<cv::Vec3b>(row, col) !=  mapColor) {

					if(PIXEL_APOVE(row, col)  && PIXEL_LEFT(row, col) ) {

						features_t tmpF;
						tmpF.y = row;
						tmpF.x = col;
						tmpF.type = featureType::INDRE;
						tmpF.loc = featureLocalation::LEFT_APOVE;
						f.push_back(tmpF); 
					}  

					if(PIXEL_APOVE(row, col) && PIXEL_RIGHT(row, col) ) { 

						features_t tmpF;
						tmpF.y = row;
						tmpF.x = col;
						tmpF.type = featureType::INDRE;
						tmpF.loc = featureLocalation::RIGHT_APOVE;
						f.push_back(tmpF);
					}

					if(PIXEL_DOWN(row, col) && PIXEL_LEFT(row, col) ) { 

						features_t tmpF;
						tmpF.y = row;
						tmpF.x = col;
						tmpF.type = featureType::INDRE;
						tmpF.loc = featureLocalation::LEFT_BELOW;
						f.push_back(tmpF);
					}

					if(PIXEL_DOWN(row,col) && PIXEL_RIGHT(row, col) )    { 

						features_t tmpF;
						tmpF.y = row;
						tmpF.x = col;
						tmpF.type = featureType::INDRE;
						tmpF.loc = featureLocalation::RIGHT_BELOW;
						f.push_back(tmpF);
					}

					// Check of outers corners

					if(PIXEL_APOVE_LEFT(row,col) && !(PIXEL_DOWN(row, col) || PIXEL_LEFT(row, col) || PIXEL_RIGHT(row, col) || PIXEL_APOVE(row, col)) ) { 
						features_t tmpF;
						tmpF.y = row;
						tmpF.x = col;
						tmpF.type = featureType::OUTER;
						tmpF.loc = LEFT_APOVE;
						f.push_back(tmpF);
					}

					if(PIXEL_APOVE_RIGHT(row,col) && !(PIXEL_DOWN(row, col) || PIXEL_LEFT(row, col) || PIXEL_RIGHT(row, col) || PIXEL_APOVE(row, col))) { 

						features_t tmpF;
						tmpF.y = row;
						tmpF.x = col;
						tmpF.type = featureType::OUTER;
						tmpF.loc = featureLocalation::RIGHT_APOVE;
						f.push_back(tmpF);
					}

					if(PIXEL_DOWN_LEFT(row,col)&& !(PIXEL_DOWN(row, col) || PIXEL_LEFT(row, col) || PIXEL_RIGHT(row, col) || PIXEL_APOVE(row, col))) { 

						features_t tmpF;
						tmpF.y = row;
						tmpF.x = col;
						tmpF.type = featureType::OUTER;
						tmpF.loc = featureLocalation::LEFT_BELOW;
						f.push_back(tmpF);
					}

					if(PIXEL_DOWN_RIGHT(row,col)&& !(PIXEL_DOWN(row, col) || PIXEL_LEFT(row, col) || PIXEL_RIGHT(row, col) || PIXEL_APOVE(row, col)) ) {

						features_t tmpF;
						tmpF.y = row;
						tmpF.x = col;
						tmpF.type = featureType::OUTER;
						tmpF.loc = featureLocalation::RIGHT_BELOW;
						f.push_back(tmpF);
					} 
				}  
			}
		}

		find_connections(f); 
	} 



	void featureExtractorCornor(const cv::Mat & map, cv::Mat & dst, std::vector<features_t> & f, cv::Vec3b mapColor) {

		int rows = map.rows;
		int cols = map.cols;
		dst = map.clone(); 

		for(int row = 1; row < rows - 1  ;  row++)
		{
			for(int col = 1; col < cols -1  ; col++ )
			{
				if(map.at<cv::Vec3b>(row, col) !=  mapColor) {

					if(PIXEL_APOVE(row, col)  && PIXEL_LEFT(row, col) ) {

						dst.at<cv::Vec3b>(row,col) = cv::Vec3b(0, 255, 0);  

						features_t tmpF;
						tmpF.y = row;
						tmpF.x = col;
						tmpF.type = featureType::INDRE;
						tmpF.loc = featureLocalation::LEFT_APOVE;
						f.push_back(tmpF); 
					}  

					if(PIXEL_APOVE(row, col) && PIXEL_RIGHT(row, col) ) { 
						dst.at<cv::Vec3b>(row,col) = cv::Vec3b(0, 255, 0);  

						features_t tmpF;
						tmpF.y = row;
						tmpF.x = col;
						tmpF.type = featureType::INDRE;
						tmpF.loc = featureLocalation::RIGHT_APOVE;
						if(f[f.size()-1].loc == featureLocalation::LEFT_APOVE) {

							f[f.size() - 1].connected[f[f.size() - 1].numberOfConnected++] = f.size();  
							tmpF.connected[tmpF.numberOfConnected++] = f.size()-1; 

						}  
						f.push_back(tmpF);
					}

					if(PIXEL_DOWN(row, col) && PIXEL_LEFT(row, col) ) { 
						dst.at<cv::Vec3b>(row,col) = cv::Vec3b(0, 255, 0);  

						features_t tmpF;
						tmpF.y = row;
						tmpF.x = col;
						tmpF.type = featureType::INDRE;
						tmpF.loc = featureLocalation::LEFT_BELOW;
						f.push_back(tmpF);
					}

					if(PIXEL_DOWN(row,col) && PIXEL_RIGHT(row, col) )    { 
						dst.at<cv::Vec3b>(row,col) = cv::Vec3b(0, 255, 0);  

						features_t tmpF;
						tmpF.y = row;
						tmpF.x = col;
						tmpF.type = featureType::INDRE;
						tmpF.loc = featureLocalation::RIGHT_BELOW;
						if(f[f.size()-1].loc == featureLocalation::LEFT_BELOW) {

							f[f.size() - 1].connected[f[f.size() - 1].numberOfConnected++] = f.size();  
							tmpF.connected[tmpF.numberOfConnected++] = f.size()-1; 

						}  
						f.push_back(tmpF);
					}

					// Check of outers corners

					if(PIXEL_APOVE_LEFT(row,col) && !(PIXEL_DOWN(row, col) || PIXEL_LEFT(row, col) || PIXEL_RIGHT(row, col) || PIXEL_APOVE(row, col)) ) { 
						dst.at<cv::Vec3b>(row,col) = cv::Vec3b(255, 0, 0); 

						features_t tmpF;
						tmpF.y = row;
						tmpF.x = col;
						tmpF.type = featureType::OUTER;
						tmpF.loc = featureLocalation::LEFT_APOVE;

						if(f[f.size()-1].loc == featureLocalation::RIGHT_APOVE) {

							f[f.size() - 1].connected[f[f.size() - 1].numberOfConnected++] = f.size();  
							tmpF.connected[tmpF.numberOfConnected++] = f.size()-1; 

						}
						f.push_back(tmpF);
					}

					if(PIXEL_APOVE_RIGHT(row,col) && !(PIXEL_DOWN(row, col) || PIXEL_LEFT(row, col) || PIXEL_RIGHT(row, col) || PIXEL_APOVE(row, col))) { 
						dst.at<cv::Vec3b>(row,col) = cv::Vec3b(255, 0, 0);  

						features_t tmpF;
						tmpF.y = row;
						tmpF.x = col;
						tmpF.type = featureType::OUTER;
						tmpF.loc = featureLocalation::RIGHT_APOVE;

						f.push_back(tmpF);
					}

					if(PIXEL_DOWN_LEFT(row,col)&& !(PIXEL_DOWN(row, col) || PIXEL_LEFT(row, col) || PIXEL_RIGHT(row, col) || PIXEL_APOVE(row, col))) { 
						dst.at<cv::Vec3b>(row,col) = cv::Vec3b(255, 0, 0);  

						features_t tmpF;
						tmpF.y = row;
						tmpF.x = col;
						tmpF.type = featureType::OUTER;
						tmpF.loc = featureLocalation::LEFT_BELOW;
						if(f[f.size()-1].loc == featureLocalation::RIGHT_BELOW) {

							f[f.size() - 1].connected[f[f.size() - 1].numberOfConnected++] = f.size();  
							tmpF.connected[tmpF.numberOfConnected++] = f.size()-1; 

						}
						f.push_back(tmpF);
					}

					if(PIXEL_DOWN_RIGHT(row,col)&& !(PIXEL_DOWN(row, col) || PIXEL_LEFT(row, col) || PIXEL_RIGHT(row, col) || PIXEL_APOVE(row, col)) ) {
						dst.at<cv::Vec3b>(row,col) = cv::Vec3b(255, 0, 0);  

						features_t tmpF;
						tmpF.y = row;
						tmpF.x = col;
						tmpF.type = featureType::OUTER;
						tmpF.loc = featureLocalation::RIGHT_BELOW;

						f.push_back(tmpF);
					} 
				}  
			}
		}

		find_connections(f); 
	} 


	void featureExtractorCornor(const std::vector<line> & lines, std::vector<features_t> & f, const double threshold) {
		//		std::cout << "Enter feature featureExtractorCornor " << threshold << std::endl; 

		if(lines.size() == 0)	
			throw "Empty"; 

		vec2 points[2];
		vec2 point[2];

		//		std::cout << lines.size() << std::endl; 
		for(size_t i = 0; i < lines.size()-1; i++ )
		{

			if(lines[i].is_orgotonalt_to(lines[i+1], 1)) {

				//std::cout << "Index " << i << std::endl;  
				//std::cout << "The list is orgotonalt "<< std::endl; 
				lines[i+1].get_point_to_draw_the_line(points[0], points[1]); 	
				lines[i].get_point_to_draw_the_line(point[0], point[1]); 	

				double distance1, distance2, distance3, distance4;

				//				std::cout << "current line " << point[0] << " " << point[1] << std::endl;   
				//				std::cout << "next line "  <<points[0] << " " << points[1] << std::endl;  
				distance1 = ((vec2)(points[0] - point[0])).length(); 
				distance2 = ((vec2)(points[1] - point[0])).length(); 
				distance3 = ((vec2)(points[0] - point[1])).length(); 
				distance4 = ((vec2)(points[1] - point[1])).length(); 


				//				std::cout << distance1  << " " << distance2  << " " << distance3 << " " << distance4  << std::endl;  

				bool flag = distance1 < threshold || distance2 < threshold || distance3 < threshold || distance4 < threshold ? true : false; 

				if(flag) {

					line tmp(point[0], points[1], pointType::CARTESIAN); 

					double d = tmp.distance_to_point(points[0]); 

					features_t tmpF;
					tmpF.x = points[0].getX();
					tmpF.y = points[0].getY(); 
					tmpF.index = i;



					if(d < 0 ) { 

						tmpF.type = featureType::OUTER;
					//							std::cout << "outer" << std::endl; 	
					} else {  
						tmpF.type = featureType::INDRE;
					//							std::cout << "indre" << std::endl; 
					}

					if(f.size() > 0 ) {

						int last = f.size()-1; 
						if(f[last].index != -1 && f[last].index + 1 == (int) i ) {

							f[last].connected[f[last].numberOfConnected++] = f.size();
							tmpF.connected[tmpF.numberOfConnected++] = last;	

						}  

					}
					f.push_back(tmpF);
					//					std::cout << "Feature extracted from " << i << " and " << i+1 << std::endl;  	

				}  

			}  

		}


	}  

} 
