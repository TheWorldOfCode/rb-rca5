#include "../includes/roadmap.hpp"

#include "../includes/brushfire.hpp" 
#include "../includes/featureExtractor.hpp" 

#include <limits> 
#include <cstdlib>


roadmap::roadmap(const cv::Mat & brushfir, const int max_value) noexcept {
	roads = cv::Mat::zeros(brushfir.rows, brushfir.cols, CV_32S);
	gerenate_GVD(brushfir, max_value); 

}  


void roadmap::remove_nodes(std::vector<pixel> & nodes, const pixel p) const noexcept {

	for(size_t i = 0; i < nodes.size(); i++ )
	{
		if(nodes[i] == p) 
			nodes.erase(nodes.begin() + i ); 

	}


}   

void roadmap::gerenate_GVD(const cv::Mat & brushfire, const int max_value) {

	roadMap = brushfire.clone();
	cv::Mat contourImage(roadMap.rows, roadMap.cols, CV_8UC1) ;
	contourImage.setTo(0); 

	std::vector<pixel> nodes;
	gerenate_GVD1(brushfire, nodes, max_value);
	gerenate_GVD2(nodes);
	for(const pixel p : nodes )
	{
	    contourImage.at<uchar>(p.row, p.col) = 255;  
	}

	gerenate_GVD2(brushfire, contourImage, nodes);
	remove_duplicate_nodes(nodes);
#if DEBUG_ROADMAP == 1
	for(const pixel p : nodes )
	{
	    contourImage.at<uchar>(p.row, p.col) = 255;  
	}
	

	cv::imwrite("../test/test.png", contourImage ); 
#endif

    cv::imwrite("test.png", roadMap);
	gerenate_GVD3(nodes);

#if DEBUG_ROADMAP == 1

	std::cout << nodes.size() << std::endl; 
	cv::Mat image2(roadMap.rows, roadMap.cols, CV_8UC1) ;
	contourImage.setTo(0); 
	for(const pixel p : nodes )
	{
	    image2.at<uchar>(p.row, p.col) = 255;  
	}
	

	cv::imwrite("../test/test2.png", image2); 
	cv::imwrite("../test/mapRoad.png", roadMap); 
#endif

}  


void roadmap::draw_roadmap(cv::Mat & dst) const noexcept { 

	dst = cv::Mat(roadMap.rows, roadMap.cols, CV_8UC3); 
	dst.setTo(255); 

#if DEBUG_ROADMAP == 1
	for(const intersection i : graph)
	{
		std::cout << print(i) << std::endl;  

	}

	for(const road r : roadsGraph )
	{
		std::cout << print(r) << std::endl;  
	}
#endif



	for(const road r : roadsGraph )
		//	for(size_t i = 0; i < 1 /*roadsGraph.size()*/ ; i++) 
	{
		//	const road r = roadsGraph[i];

		if(r.pt1 != -1 && r.pt2 != -1) {  
			const intersection i1 = graph[r.pt1];
			const intersection i2 = graph[r.pt2];
			dst.at<cv::Vec3b>(i1.point.row, i1.point.col) = cv::Vec3b(255,0,0);   
			dst.at<cv::Vec3b>(i2.point.row, i2.point.col) = cv::Vec3b(255,0,0);   


			explore_draw(dst, i1.point, i1.point, r.dir, i2.point);  

		}
	}
}  

void roadmap::gradient(const cv::Mat & src, cv::Mat & angleDst, cv::Mat & magDst, std::vector< std::tuple<float, cv::Vec3b>> & angle, std::vector< std::tuple<float, cv::Vec3b>> & mag) {

	angle.clear();
	mag.clear(); 	

	magDst = src.clone(); 
	angleDst = src.clone(); 

	cv::Mat gray;
	cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY); 

	gray.convertTo(gray, CV_32F, 1/255.0);

	cv::Mat dx, dy;
	cv::Sobel(gray, dx, CV_32F, 1,0, 1 );
	cv::Sobel(gray, dy, CV_32F,0,1, 1 );


	cv::Mat angle2, mag2;


	cv::cartToPolar(dx,dy,mag2,angle2,1); 

	for(int row = 0 ; row < angle2.rows ; row++ )
	{
		for(int col = 0 ; col < angle2.cols; col++  )
		{
			if(src.at<cv::Vec3b>(row,col) == cv::Vec3b(0,0,0)) 
				continue;

			bool flag = true;
			for(const std::tuple<float,cv::Vec3b> f1 : angle )
			{

				if(angle2.at<float>(row,col) == std::get<0>(f1)) {  
					flag = false;
					angleDst.at<cv::Vec3b>(row,col) = std::get<1>(f1); 
				}
			}

			if(flag) {  
				cv::Vec3b color = cv::Vec3b( std::rand() % 255, std::rand() % 255, std::rand() % 255    );
				angle.push_back(std::tuple<float, cv::Vec3b>(angle2.at<float>(row,col), color)); 
				angleDst.at<cv::Vec3b>(row,col) = color;
			}

			bool flag2 = true;
			for(const std::tuple<float,cv::Vec3b> f1 : mag )
			{

				if(angle2.at<float>(row,col) == std::get<0>(f1)) {  
					flag2 = false;
					magDst.at<cv::Vec3b>(row,col) = std::get<1>(f1); 
				}
			}

			if(flag2) {  
				cv::Vec3b color = cv::Vec3b( std::rand() % 255, std::rand() % 255, std::rand() % 255    );
				mag.push_back(std::tuple<float, cv::Vec3b>(mag2.at<float>(row,col), color)); 
				magDst.at<cv::Vec3b>(row,col) = color;
			}

		}


	}



	std::vector< std::tuple<float, cv::Vec3b>> angleNew;

	for(const std::tuple<float, cv::Vec3b> infor : angle )
	{
		if(angleNew.size() != 0) {

			const float tmp = std::get<0>(infor); 

			for(size_t i = 0; i < angleNew.size(); i++  )
			{
				if(std::get<0>(angleNew[i] ) > tmp ) {
					angleNew.insert(angleNew.begin() + i, infor  );
					break;
				}   

				if(i == angleNew.size() - 1 ) { 
					angleNew.push_back(infor); 
					break;
				}
			}




		} else
			angleNew.push_back(infor);    




	}

	angle.clear(); 
	angle = angleNew;	


}
roadmap::~roadmap() {}  


void roadmap::gerenate_GVD1(const cv::Mat & brushfire, std::vector<pixel> & nodes, const int max_value) {

	const int rows = roadMap.rows;
	const int cols = roadMap.cols;

	for(int row = 0; row < rows ; row++ )
	{
		for(int col = 0; col < cols; col++ )
		{
			uchar i = brushfire.at<cv::Vec3b>(row,col)[0]; 
			if(i == 0) { 
				//			angle2.at<float>(row,col) = -1; 
				continue;	
			}

			bool flag;
			flag = detect_indre_cornor(brushfire, row, col, cv::Vec3b(0,0,0)); 


			if(flag) {
				nodes.push_back((pixel) {row, col}   );
				create_intersection((pixel) {row,col} ); 
				continue;
			}  

			flag = detect_indre_cornor(brushfire, row, col, cv::Vec3b(i-BRUSHFIRE_STEP_SIZE,i - BRUSHFIRE_STEP_SIZE,i - BRUSHFIRE_STEP_SIZE)); 

			if(flag) {
				nodes.push_back((pixel) {row, col});
				roadMap.at<cv::Vec3b>(row,col) = cv::Vec3b(0,255,0);  
				continue;
			}  

			int n = check_neighbors(brushfire,i, row, col);  // Same index as the current pixel	
			int n2;

			if(i != BRUSHFIRE_BEGIN) 
				n2 = check_neighbors(brushfire,i - BRUSHFIRE_STEP_SIZE, row,col);  // One lower then th current pixel
			else 
				n2 = check_neighbors(brushfire,0,row,col);	

			int n3 = check_neighbors(brushfire,i + BRUSHFIRE_STEP_SIZE, row, col); // One high then the current pixel

			if(i == max_value && brushfire.at<cv::Vec3b>(row, col)  == cv::Vec3b((uchar) max_value, (uchar) max_value, (uchar) max_value ) ) {

				roadMap.at<cv::Vec3b>(row,col) = cv::Vec3b(0,255,0);  
				nodes.push_back((pixel) {row, col}   );

			} else {  

				if(n3 == 0 && n == 6 && n2 == 3) { 
					roadMap.at<cv::Vec3b>(row,col) = cv::Vec3b(0, 255, 0);  	
					nodes.push_back((pixel) {row, col}   );

				} else 	if(n3 == 0 && n == 5 && n2 == 4) { 
					roadMap.at<cv::Vec3b>(row,col) = cv::Vec3b(0, 255, 0);  	
					nodes.push_back((pixel) {row, col}   );
				} else 	if(n3 == 0 && n2 > 4 && n < 5) { 
					roadMap.at<cv::Vec3b>(row,col) = cv::Vec3b(0,255,0);  
					nodes.push_back((pixel) {row, col}   );
				} else if(n3 == 0 && n2 > 3) {  
					if (n == 2 || n == 3) { 
						roadMap.at<cv::Vec3b>(row,col) = cv::Vec3b(0,225,0);  
						nodes.push_back((pixel) {row, col}   );
					}
				} else if(n3 == 0 && n2 < 3 && n > 4) { 
					roadMap.at<cv::Vec3b>(row,col) = cv::Vec3b(0,255,0);  
					nodes.push_back((pixel) {row, col}   );
				} else if(n3 <= 2 && n == 3 && n2 > 3)  { 
					roadMap.at<cv::Vec3b>(row,col) = cv::Vec3b(0,225,0);
					nodes.push_back((pixel) {row, col}   );
				}


			}
		}


	}


}   

void roadmap::gerenate_GVD2(std::vector<pixel> & nodes) {

	for(const pixel p : nodes )
	{
		if(roadMap.at<cv::Vec3b>(p.row, p.col) == cv::Vec3b(0,0,255)  ) 
			continue;

		int n = check_neighbors(roadMap, cv::Vec3b(255,0,0), p.row, p.col)
			+ check_neighbors(roadMap, cv::Vec3b(0,255,0), p.row, p.col) 
			+ check_neighbors(roadMap, cv::Vec3b(255,255,0), p.row, p.col) ;

		int n2 = check_neighbors(roadMap, cv::Vec3b(0,0,0), p.row, p.col);

		if(n == 2 && n2 < 4) { 

			std::vector<direction> green, blue, light_blue, black;
			int r = find_moving_directions(roadMap, light_blue, p, cv::Vec3b(255,255,0))
				+ find_moving_directions(roadMap, green, p, cv::Vec3b(0,255,0)) 
				+ find_moving_directions(roadMap, blue, p, cv::Vec3b(255,0,0));

			roadMap.at<cv::Vec3b>(p.row, p.col) = cv::Vec3b(255,255,0);   
			explore_directions(roadMap, p, green, nodes);  
			if( roadMap.at<cv::Vec3b>(p.row, p.col) == cv::Vec3b(255,255,0) )
				roadMap.at<cv::Vec3b>(p.row, p.col) = cv::Vec3b(0,255,0);   

		}
	}

}  

void roadmap::gerenate_GVD2(const cv::Mat & brushfire, const cv::Mat & contourImage, std::vector<pixel> & nodes) {

	cv::Mat contourImage2 = contourImage.clone() ;
	std::vector<cv::Vec4i> Hier;
	std::vector<std::vector<cv::Point>> contour;
	cv::threshold(contourImage2 , contourImage2, 255/2, 255, CV_THRESH_BINARY);
	cv::findContours(contourImage2, contour, Hier, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	int base = 0;
	size_t max = 0;

	for(size_t i = 0; i < contour.size(); i++ )
	{

		if(contour[i].size() > max ) {

			max = contour[i].size();
			base = i;	
		} 
	}


	double distance_min = std::numeric_limits<double>::max(); 
	cv::Point s1;
	cv::Point s2;

	int x = 0;
	int i = 0;
	int old = contour.size()+1;
	while( ((size_t) old != contour.size() || x < 10) && contour.size() != 1  ) {
		old = contour.size();
		std::vector<cv::Point> n2;
		bool flag = false;
		for(const cv::Point p1 : contour[i] )
		{
			for(const cv::Point p2 : contour[base])
			{
				if(p2 != p1) {
					double distance = (p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y);
					if(distance < 500) {
						std::vector<cv::Point> n1;
						if(brushfire.at<cv::Vec3b>(p1.y,p1.x)[0] < brushfire.at<cv::Vec3b>(p2.y,p2.x)[0]  )
							flag = move(n1, brushfire, roadMap, p1, p2);
						else
							flag = move(n1, brushfire, roadMap, p2, p1);

						if(flag == true) {
							if(distance < distance_min) {

								distance_min = distance;
								n2.clear(); 
								n2 = n1;

							}  

							n1.clear();
						} 


					}  
				}

			}

		}


		if(n2.size() != 0 ) { 

			move_draw(n2, nodes, roadMap); 	
			for(const cv::Point p : contour[i] )
			{
				contour[base].push_back(p);  
			}

			for(const cv::Point p : n2 )
			{
				contourImage2.at<uchar>(p.y,p.x) = 255; 
				contour[base].push_back(p); 
			}

			contour.erase(contour.begin() + i );
			Hier.erase(Hier.begin() + i ); 

			if( i < base) 
				base--;
		}
		else 
			i++;

		n2.clear(); 
		distance_min = std::numeric_limits<double>::max(); 

		if(i == base)
			i++;	

		if((size_t) i == contour.size() ) {
			i = 0;
			x++;

		} 

		if(i == base)
			i++;	



	}  

#if DEBUG_ROADMAP == 1
	contour.clear();
        Hier.clear(); 	

	cv::threshold(contourImage2 , contourImage2, 255/2, 255, CV_THRESH_BINARY); 
	cv::findContours(contourImage2, contour, Hier, CV_RETR_LIST, CV_CHAIN_APPROX_NONE); 
	base = 0;
	max = 0;

	for(size_t i = 0; i < contour.size(); i++ )
	{

		if(contour[i].size() > max ) {

			max = contour[i].size();
			base = i;	
		} 
	}

	cv::Mat left(contourImage.rows, contourImage.cols, CV_8UC3); 
	left.setTo(0); 
	for(size_t i = 0; i < contour.size(); i++  )
	{
		cv::Vec3b color; 
		if((int) i == base)
			color = cv::Vec3b(0, 255,0); 
		else
			color = cv::Vec3b(255 % std::rand(), 0, 255 & std::rand()); 

		cv::drawContours(left, contour,i,color,1, cv::LINE_8,Hier); 

	}

	cv::imwrite("../test/leftOfContours.png", left ); 



#endif 

}  


void roadmap::gerenate_GVD3(std::vector<pixel> & nodes) {


	size_t size = nodes.size();
	size_t size_new = 0;
	while(size != size_new) { 
		size = size_new;
		for(const pixel p : nodes )
		{
			if(roadMap.at<cv::Vec3b>(p.row, p.col) == cv::Vec3b(255,0,0)) {
				std::vector<direction> dir_list;
				find_moving_directions(roadMap, dir_list,p, cv::Vec3b(0,255,0)); 

				for(const direction dir : dir_list )
					create_road(roadMap, dir, p, p,nodes, -1);

				remove_nodes(nodes, p); 

			}
		}

		size_new = nodes.size(); 
	}

}  

void roadmap::mark_road(const pixel p) {
	if(roadMap.at<cv::Vec3b>(p.row, p.col) != cv::Vec3b(0,0, 255) && roadMap.at<cv::Vec3b>(p.row, p.col) != cv::Vec3b(255,0,0)) {  
		roadMap.at<cv::Vec3b>(p.row, p.col) = cv::Vec3b(0,0, 255);  
		roads.at<int>(p.row, p.col) = (int)roadsGraph.size();
	}
}  

int roadmap::new_road(const int start, const direction dir) {

	roadsGraph.push_back(road(start, -1, dir, roadsGraph.size())); 
	graph[start].add_road(roadsGraph.size() - 1); 

	return roadsGraph.size() - 1; 
}  


void roadmap::connect_to_road(const int intersection_number, const int road_number) {

	try {
	    float distance = -1;
	    if(roadsGraph[road_number].pt1 != -1) {
            pixel it1 = graph[roadsGraph[road_number].pt1].point;
            pixel it2 = graph[intersection_number].point;
	        distance = sqrt((it1.col-it2.col)*(it1.col-it2.col) + (it1.row-it2.row)*(it1.row-it2.row)); //////////////////////////////////////////////////////////////////////////
	    }

		roadsGraph[road_number].connect(intersection_number, distance );
		graph[intersection_number].add_road(road_number); 
		remove_duplicate_road(intersection_number, road_number); 


	} catch (FullConnectedRoad r) {

		std::cout << "tried to add intersection " << intersection_number << " at (" << graph[intersection_number].point.row << ", "  << graph[intersection_number].point.col << ") to road " << road_number << std::endl; 


	}    


} 

void roadmap::add_road(const int intersection_number1, const int intersection_number2, const int road_number) {

	std::cout << "Not implement yet" << std::endl; 

}   

void roadmap::delete_road(const int road_number) {

	graph[roadsGraph[road_number].pt1].remove_road(road_number); 
	graph[roadsGraph[road_number].pt2].remove_road(road_number); 

	roadsGraph[road_number].remove_road(); 
	//	roadsGraph.erase(roadsGraph.begin() + road_number); 

} 


void roadmap::remove_duplicate_road(const int intersection_number, const int road_number) {

	for( const int r : graph[intersection_number].road_numbers )
	{
		if(roadsGraph[r] == roadsGraph[road_number] && r != road_number) {
			delete_road(road_number); 
			break;
		}   
	}

}  

int roadmap::create_intersection(const pixel point)  {

	if(roadMap.at<cv::Vec3b>(point.row, point.col)  == cv::Vec3b(255,0,0)) {
		return roads.at<int>(point.row,point.col); 
	} else { 	
		roadMap.at<cv::Vec3b>(point.row, point.col) = cv::Vec3b(255,0,0);

		graph.push_back(intersection(point, graph.size())); 
		roads.at<int>(point.row, point.col) = graph.size() - 1; 

		return graph.size()-1; 
	}
} 

void roadmap::connect_to_intersection(const int new_intersection, const direction dir, const pixel p, bool use_dir) {

	pixel p_new(-1,-1) ; 

	if(use_dir) {
		switch(dir) {
			case LEFT:
				p_new = pixel( p.row, p.col - 1); 
				break;
			case LEFT_UP:
				p_new = pixel(p.row - 1, p.col - 1); 
				break;
			case UP:
				p_new = pixel(p.row - 1, p.col); 
				break;
			case RIGHT_UP:
				p_new = pixel(p.row - 1, p.col + 1); 
				break;
			case RIGHT:
				p_new = pixel(p.row, p.col + 1); 
				break;
			case RIGHT_DOWN:
				p_new = pixel(p.row + 1, p.col + 1); 
				break;
			case DOWN:
				p_new = pixel(p.row + 1, p.col); 
				break;
			case LEFT_DOWN:
				p_new = pixel(p.row + 1, p.col - 1); 
				break;
		}  
	}

	const int old_intersection = roads.at<int>(p_new.row, p_new.col); 

	if(roadMap.at<cv::Vec3b>(p_new.row, p_new.col) != cv::Vec3b(255,0,0)  ) 
		std::cout << "Something wrong" << std::endl; 

	int road_new = new_road(new_intersection,dir); 
	connect_to_road(old_intersection, road_new); 
}  

void roadmap::split_road(const direction dir, const int intersection_number) {
    std::cout << "Enter split road paremeter " << print(dir) << " " << intersection_number << std::endl;
    pixel p_new(-1,-1);
    const pixel p = graph[intersection_number].point;
        switch(dir) {
            case LEFT:
                p_new = pixel( p.row, p.col - 1);
                break;
            case LEFT_UP:
                p_new = pixel(p.row - 1, p.col - 1);
                break;
            case UP:
                p_new = pixel(p.row - 1, p.col);
                break;
            case RIGHT_UP:
                p_new = pixel(p.row - 1, p.col + 1);
                break;
            case RIGHT:
                p_new = pixel(p.row, p.col + 1);
                break;
            case RIGHT_DOWN:
                p_new = pixel(p.row + 1, p.col + 1);
                break;
            case DOWN:
                p_new = pixel(p.row + 1, p.col);
                break;
            case LEFT_DOWN:
                p_new = pixel(p.row + 1, p.col - 1);
                break;
        }

    const int road_number = roads.at<int>(p_new.row, p_new.col) - 1;

    std::cout << print(graph[intersection_number]) << std::endl;
    std::cout << print(roadsGraph[road_number]) << std::endl;
    /*
    const int unconnected_intersection = roadsGraph[road_number].pt2;
    roadsGraph[road_number].change(unconnected_intersection, intersection_number);
    graph[intersection_number].add_road(road_number);
    remove_duplicate_road(intersection_number, road_number);

    std::cout << print(graph[intersection_number]) << " " << print(graph[unconnected_intersection])
                << intersection_number << " " << unconnected_intersection << std::endl;
    direction dir2 = find_direction(graph[intersection_number].point, graph[unconnected_intersection].point);
    int road_new = new_road(intersection_number, dir2);
    std::cout << road_number << " " << road_new << std::endl;
    std::cout << print(roadsGraph[road_new]) << std::endl;
    graph[unconnected_intersection].remove_road(road_number);

    connect_to_road(unconnected_intersection, road_new);
    */
     const int pt1 = roadsGraph[road_number].pt1;
     const int pt2 = roadsGraph[road_number].pt2;
    direction dir2 = find_direction(graph[intersection_number].point, graph[pt1].point);
    direction dir3 = find_direction(graph[intersection_number].point, graph[pt2].point);

    int road_new = new_road(intersection_number, dir2);
    int road_new2 = new_road(intersection_number,dir3);
    connect_to_road(pt1, road_new);
    connect_to_road(pt2, road_new2);
     std::cout << "Exit" << std::endl;
    std::cout << print(graph[intersection_number]) << std::endl;
    std::cout << print(roadsGraph[road_number]) << std::endl;
    std::cout << print(roadsGraph[road_new]) << std::endl;
    std::cout << print(roadsGraph[road_new2]) << std::endl;
    std::cout << print(graph[pt1]) << std::endl;
    std::cout << print(graph[pt2]) << std::endl;
    std::cout << std::endl << std::endl;
}  

#define CHECK_APOVE src.at<float>(row - 1, col)  
#define CHECK_LEFT_APOVE src.at<float>(row - 1, col-1)  
#define CHECK_RIGHT_APOVE src.at<float>(row - 1, col+1)  
#define CHECK_LEFT src.at<float>(row, col - 1)  
#define CHECK_RIGHT src.at<float>(row, col +1)  
#define CHECK_DOWN src.at<float>(row + 1, col)  
#define CHECK_LEFT_DOWN src.at<float>(row + 1, col-1)  
#define CHECK_RIGHT_DOWN src.at<float>(row + 1, col+1)  
// BiCube
int roadmap::gradient2(const cv::Mat & brushfire, const cv::Mat & src, const int row, const int col) {

	float apove_left = CHECK_LEFT_APOVE;
	float apove = CHECK_APOVE;
	float apove_right = CHECK_RIGHT_APOVE;

	float center_left = CHECK_LEFT;
	float center = src.at<float>(row,col); 
	float center_right = CHECK_RIGHT;

	float down_left = CHECK_LEFT_DOWN;
	float down = CHECK_DOWN;
	float down_right = CHECK_RIGHT_DOWN;


	float sum = 0;

	if(apove_left != -1) 
		sum += apove_left;

	if(apove != -1)
		sum += apove;

	if(apove_right != -1)
		sum += apove_right;	

	if(center_left != -1)
		sum += center_left;


	if(center != -1)
		sum += center;


	if(center_right != -1)
		sum += center_right;


	if(down_left != -1)
		sum += down_left;


	if(down != -1)
		sum += down;

	if(down_right != -1)
		sum += down_right;	


	return (int) std::ceil(sum) % 360;


}  

int roadmap::check_neighbors(const cv::Mat & map, const uchar value, const int row, const int col) { return check_neighbors(map, cv::Vec3b(value,value,value), row, col); }  

int roadmap::check_neighbors(const cv::Mat & map, const cv::Vec3b value, const int row, const int col) {


	int count = 0;
	for(int r = row - 1; r < row + 2; r++ )
	{
		if(r > -1 && r < map.rows) {  

			for(int c = col - 1; c < col + 2 ; c++ )
			{
				if(c > -1 && c < map.cols) {

					if(map.at<cv::Vec3b>(r,c) == value) {
						count++;
					} 

				}  

			}
		}
	}

	return count;

} 



#define PIXEL_APOVE(row, col) map.at<cv::Vec3b>(row - 1, col) == mapColor  
#define PIXEL_LEFT(row, col)  map.at<cv::Vec3b>(row, col - 1) == mapColor 
#define PIXEL_RIGHT(row, col) map.at<cv::Vec3b>(row, col + 1) == mapColor   
#define PIXEL_DOWN(row, col)  map.at<cv::Vec3b>(row + 1, col) == mapColor

#define PIXEL_APOVE_RIGHT(row, col) map.at<cv::Vec3b>(row - 1, col + 1) == mapColor   
#define PIXEL_APOVE_LEFT(row, col)  map.at<cv::Vec3b>(row - 1, col - 1) == mapColor   
#define PIXEL_DOWN_RIGHT(row, col)  map.at<cv::Vec3b>(row + 1, col + 1) == mapColor   
#define PIXEL_DOWN_LEFT(row, col)   map.at<cv::Vec3b>(row + 1, col - 1) == mapColor   



bool roadmap::detect_indre_cornor(const cv::Mat & map, const int row, const int col, const cv::Vec3b mapColor) {

	if(PIXEL_APOVE(row, col)  && PIXEL_LEFT(row, col) ) {

		roadMap.at<cv::Vec3b>(row,col) = cv::Vec3b(0,255,0);  
		return true;
	}  

	if(PIXEL_APOVE(row, col) && PIXEL_RIGHT(row, col) ) { 

		roadMap.at<cv::Vec3b>(row,col) = cv::Vec3b(0,255,0);  
		return true;
	}

	if(PIXEL_DOWN(row, col) && PIXEL_LEFT(row, col) ) { 

		roadMap.at<cv::Vec3b>(row,col) = cv::Vec3b(0,255,0);  
		return true;
	}

	if(PIXEL_DOWN(row,col) && PIXEL_RIGHT(row, col) )    { 
		roadMap.at<cv::Vec3b>(row,col) = cv::Vec3b(0,255,0);  
		return true;
	}


	return false;

}

roadmap::direction roadmap::find_direction(const pixel from, const pixel to) {
    pixel tmp(to.row - from.row,to.col - from.col);
    tmp.row /= std::abs(tmp.row);
    tmp.col /= std::abs(tmp.col);

    std::cout << print(from) << " " << print(to) << " " << print(tmp) << "THIS" << std::endl;
    if(tmp == pixel(-1,0))
        return UP;

    if(tmp == pixel(-1, -1))
        return LEFT_UP;

    if(tmp == pixel(0,-1))
        return LEFT;

    if(tmp == pixel(1,-1))
        return LEFT_DOWN;

    if(tmp == pixel(1,0))
        return DOWN;

    if(tmp == pixel(1,1))
        return RIGHT_DOWN;

    if(tmp == pixel(0,1))
        return RIGHT;

    if(tmp == pixel(-1,1))
        return RIGHT_UP;
}

int roadmap::find_moving_directions(const cv::Mat & map, std::vector<direction> & dir_list, const pixel p, cv::Vec3b color) const noexcept {

	dir_list.clear(); 

	if(map.at<cv::Vec3b>(p.row + 1, p.col) == color )
		dir_list.push_back(DOWN); 
	// Green apove 
	if(map.at<cv::Vec3b>(p.row - 1, p.col) == color) 
		dir_list.push_back(UP);
	// Green left 
	if(map.at<cv::Vec3b>(p.row, p.col - 1) == color) 
		dir_list.push_back(LEFT);
	// Green right
	if(map.at<cv::Vec3b>(p.row, p.col + 1) == color)
		dir_list.push_back(RIGHT);
	// Green apove right
	if(map.at<cv::Vec3b>(p.row - 1, p.col + 1) == color)
		dir_list.push_back(RIGHT_UP);
	// Green down left
	if(map.at<cv::Vec3b>(p.row + 1, p.col - 1) == color)
		dir_list.push_back(LEFT_DOWN); 
	// Green apove left
	if(map.at<cv::Vec3b>(p.row - 1, p.col - 1) == color)
		dir_list.push_back(LEFT_UP);
	// Green down right
	if(map.at<cv::Vec3b>(p.row + 1, p.col + 1) == color)
		dir_list.push_back(RIGHT_DOWN); 

	return dir_list.size();
} 

int roadmap::reduce_directions(std::vector<direction> & dir_list, const direction dir) const noexcept {

	for(size_t i = 0; i < dir_list.size(); i++ )
	{

		if(dir == dir_list[i])
			dir_list.erase(dir_list.begin() +i );
		else {

			if(( dir == LEFT_UP && dir_list[i] == RIGHT_DOWN) || (dir == RIGHT_DOWN && dir_list[i] == LEFT_UP) ) 
				dir_list.erase(dir_list.begin() + i ); 
			else if ((dir == RIGHT_UP && dir_list[i] == LEFT_DOWN) || (dir == LEFT_DOWN && dir_list[i] == RIGHT_UP)  ) 
				dir_list.erase(dir_list.begin() + i ); 
			else if ((dir == LEFT && dir_list[i] == RIGHT) || (dir == RIGHT && dir_list[i] == LEFT))
				dir_list.erase(dir_list.begin() + i); 	
			else if ((dir == UP && dir_list[i] == DOWN) || (dir == DOWN && dir_list[i] == UP)  ) 
				dir_list.erase(dir_list.begin() + i); 	
		} 	
	}


	return dir_list.size(); 

} 


void roadmap::create_road(cv::Mat & map, const direction dir, const pixel p, const pixel c_p, std::vector<pixel> & nodes, const int road) noexcept {

	int _road = road;

	if(road == -1) {
		int intersection = create_intersection(p);
		_road = new_road(intersection, dir);  
#if DEBUG_ROADMAP == 1
		std::cout << print(graph[intersection]) << std::endl << std::endl; 
#endif 
	}   	

	std::vector<direction> blue, red, green;
	std::vector<direction> blue2, red2, green2;
	int n = find_moving_directions(map, blue, c_p, cv::Vec3b(255,0,0)) +  find_moving_directions(map, green, c_p, cv::Vec3b(0,255,0)) + find_moving_directions(map, red, c_p, cv::Vec3b(0,0,255));
	blue2 = blue;
	green2 = green;
	red2 = red;
	n = reduce_directions(blue, opposite_direction(dir)) + reduce_directions(red, dir) + reduce_directions(green, dir);   

	if(!(c_p.col > 0 && c_p.col < map.cols && c_p.row > 0 && c_p.row < map.rows )) 
		return;

	if(p != c_p) {  
		remove_nodes(nodes, c_p); 
		mark_road(c_p); 
	}

	if(n > 0 && c_p != p) {
        const int road_number = roads.at<int>(c_p.row, c_p.col);
		int new_intersection  = create_intersection(c_p); 
		connect_to_road(new_intersection, road);
		if(blue2.size() >  1 ) { 
			for(const direction d : blue2 )
			{
				if(opposite_direction(dir) != d ) 
					connect_to_intersection(new_intersection, d, c_p); 
			}


#if DEBUG_ROADMAP == 1
			std::cout << "Connecting to old interscept" << std::endl;
#endif
			// Create road to interscept 	
		}  

		if(green.size() > 0 ) {
#if DEBUG_ROADMAP == 1
			std::cout << "More not explored roads" << std::endl;
#endif
			for(const direction d : green )
			{
				create_road(map, d, c_p, c_p, nodes, -1); 
			}


		}  


		if(red.size()) {
#if DEBUG_ROADMAP == 1
            std::cout << "Not connected road" << std::endl;
            std::cout << red.size() << std::endl;
            map.at<cv::Vec3b>(c_p.row, c_p.col) = {255,255,0};
#endif
//            for (direction dir : red)
//                split_road(dir, new_intersection);
        }

	} else {


		if(map.at<cv::Vec3b>(c_p.row, c_p.col) == cv::Vec3b(255,0,0) && p != c_p ) {

			connect_to_road(roads.at<int>(c_p.row, c_p.col) , road ); 

#if DEBUG_ROADMAP == 1
			std::cout << "Connecting to old interscept 1" << std::endl;
#endif
			return;
		}  


		pixel p_new(-1,-1) ; 

		switch(dir) {
			case LEFT:
				p_new = pixel( c_p.row, c_p.col - 1); 
				break;
			case LEFT_UP:
				p_new = pixel(c_p.row - 1, c_p.col - 1); 
				break;
			case UP:
				p_new = pixel(c_p.row - 1, c_p.col); 
				break;
			case RIGHT_UP:
				p_new = pixel(c_p.row - 1, c_p.col + 1); 
				break;
			case RIGHT:
				p_new = pixel(c_p.row, c_p.col + 1); 
				break;
			case RIGHT_DOWN:
				p_new = pixel(c_p.row + 1, c_p.col + 1); 
				break;
			case DOWN:
				p_new = pixel(c_p.row + 1, c_p.col); 
				break;
			case LEFT_DOWN:
				p_new = pixel(c_p.row + 1, c_p.col - 1); 
				break;
		}  

		create_road(map, dir, p, p_new, nodes, _road);

	}  	
}  

void roadmap::expand(cv::Mat & map, const pixel p, const direction dir) { 
	int move_n;
	int move_row = 0;
	int move_col = 0;

	switch(dir) {
		case LEFT_UP:
			move_row = -1;
		case LEFT:
			move_col = -1;
			break;
		case RIGHT_UP:
			move_col = 1;
		case UP:
			move_row = -1;
			break;
		case RIGHT_DOWN:
			move_row = 1;
		case RIGHT:
			move_col = 1;
			break;
		case LEFT_DOWN:
			move_col = -1;
		case DOWN:
			move_row = 1;
			break;
	}

	if(p.row == 70 && p.col == 107) { 
		map.at<cv::Vec3b>(p.row + move_row, p.col + move_col) = cv::Vec3b(255,0,255);   
		return;
	}

	do { 
		map.at<cv::Vec3b>(p.row + move_row, p.col + move_col) = cv::Vec3b(0,255,255);   

		std::vector<direction> green, blue, yellow;
		move_n = find_moving_directions(map, green, (pixel) {p.row + move_row, p.col + move_col}, cv::Vec3b(0,255,0) )
			+ find_moving_directions(map, blue, (pixel) {p.row + move_row, p.col + move_col}, cv::Vec3b(255,0,0) ) ;


		if(move_row > 0)
			move_row++;
		else if(move_row < 0)  
			move_row--;

		if(move_col > 0)
			move_col++;  
		else if(move_col < 0) 
			move_col--;


	} while(move_n < 2 && !(move_row == 0 && move_col == 0) );
}


std::string roadmap::print(const direction d) const noexcept {

	switch(d) {
		case LEFT:
			return "LEFT"; 
		case LEFT_UP:
			return "LEFT_UP";
		case UP:
			return "UP";  
		case RIGHT_UP:
			return "RIGHT_UP"; 
		case RIGHT:
			return "RIGHT";  
		case RIGHT_DOWN:
			return "RIGHT_DOWN";
		case DOWN:
			return "DOWN"; 
		case LEFT_DOWN:
			return "LEFT_DOWN"; 
	}  
}  

std::string roadmap::print(const intersection i ) const noexcept   {

	std::string s = "Index: "+ std::to_string(i.index) + " Local (" + std::to_string(i.point.row) + ", " + std::to_string(i.point.col)  + ") Connected to roads: [";
	for(const int r : i.road_numbers )
	{
		s+= " "; 
		s += std::to_string(r) + ",";
	}

	s += "]"; 

	return s;

} 


std::string roadmap::print(const road r) const noexcept {

	return "Index: " + std::to_string(r.index) + " Direction " + print(r.dir) + " Intersections: [" + std::to_string(r.pt1) + ", " + std::to_string(r.pt2) + "]";          


}  

std::string roadmap::print(const pixel p) const noexcept { return "(" + std::to_string(p.row) + ", " + std::to_string(p.col) +  ")"; }  
roadmap::direction roadmap::degress2direction(const float angle) const noexcept {

	if(angle == 0 || angle == 360)
		return LEFT;	

	if(angle > 0 && angle < 90)
		return LEFT_UP;

	if(angle == 90)
		return UP;	

	if(90 < angle && angle < 180)
		return RIGHT_UP;	

	if(angle == 180)
		return RIGHT;	

	if(180 < angle && angle < 270) 
		return RIGHT_DOWN;

	if(angle == 270)
		return DOWN;


	if(270 < angle ) 
		return LEFT_DOWN;


}  

roadmap::direction roadmap::opposite_direction(const direction dir) noexcept {

	switch(dir) {
		case LEFT:
			return RIGHT; 
		case LEFT_UP:
			return RIGHT_DOWN;
		case UP:
			return DOWN;  
		case RIGHT_UP:
			return LEFT_DOWN; 
		case RIGHT:
			return LEFT;  
		case RIGHT_DOWN:
			return LEFT_UP;
		case DOWN:
			return UP;
		case LEFT_DOWN:
			return RIGHT_UP;
	}
}  




void roadmap::explore_directions(cv::Mat & map, const pixel p, const std::vector<direction> directions, std::vector<pixel> & nodes) noexcept {

	for( const direction d : directions)
	{
		direction d2 = opposite_direction(d); 

		int steps = explore_move(map, p, p, d2); 


		if(steps != -1) {
			explore_draw(map,p,p,d2, steps, nodes); 

		} 


	}



}  


int roadmap::explore_move(const cv::Mat & map, const pixel p, const pixel c_p, const direction dir) const noexcept {

	if(map.at<cv::Vec3b>(c_p.row, c_p.col) == cv::Vec3b(0,0,0))
		return 0;	

	std::vector<direction> dir_list, dir_list2; 
	int n = find_moving_directions(map, dir_list, c_p, cv::Vec3b(0,255,0) ) + find_moving_directions(map, dir_list2, c_p, cv::Vec3b(255,0,0) );


	if((n > 0 && p != c_p) )
		return 1;	

	if(map.at<cv::Vec3b>(c_p.row, c_p.col) == cv::Vec3b(0,255,0) || map.at<cv::Vec3b>(c_p.row, c_p.col) == cv::Vec3b(255,0,0)  ) 
		return 1;

	pixel p_new(-1,-1);
	switch(dir) {
		case LEFT:
			p_new = pixel( c_p.row, c_p.col - 1); 
			break;
		case LEFT_UP:
			p_new = pixel(c_p.row - 1, c_p.col - 1); 
			break;
		case UP:
			p_new = pixel(c_p.row - 1, c_p.col); 
			break;
		case RIGHT_UP:
			p_new = pixel(c_p.row - 1, c_p.col + 1); 
			break;
		case RIGHT:
			p_new = pixel(c_p.row, c_p.col + 1); 
			break;
		case RIGHT_DOWN:
			p_new = pixel(c_p.row + 1, c_p.col + 1); 
			break;
		case DOWN:
			p_new = pixel(c_p.row + 1, c_p.col); 
			break;
		case LEFT_DOWN:
			p_new = pixel(c_p.row + 1, c_p.col - 1); 
			break;
	}  



	int r =  explore_move(map, p, p_new, dir); 

	if(r == 0) 
		return 0;
	if(p == c_p)
		return r;	

	return 1 + r;

} 	

void roadmap::explore_draw(cv::Mat & map, const pixel p, const pixel c_p, const direction dir, const int moves_left, std::vector<pixel> & nodes) noexcept {

	int move = moves_left;

	if(move < 1)
		return;

	if(move == 1 && p != c_p) { 
		nodes.push_back(c_p);
		create_intersection(c_p); 
	} else {  
		map.at<cv::Vec3b>(c_p.row, c_p.col) = cv::Vec3b(0,255,0);  	     	
		nodes.push_back(c_p);
	}

	if(p != c_p) 
		move--;


	pixel p_new(-1,-1);
	switch(dir) {
		case LEFT:
			p_new = pixel( c_p.row, c_p.col - 1); 
			break;
		case LEFT_UP:
			p_new = pixel(c_p.row - 1, c_p.col - 1); 
			break;
		case UP:
			p_new = pixel(c_p.row - 1, c_p.col); 
			break;
		case RIGHT_UP:
			p_new = pixel(c_p.row - 1, c_p.col + 1); 
			break;
		case RIGHT:
			p_new = pixel(c_p.row, c_p.col + 1); 
			break;
		case RIGHT_DOWN:
			p_new = pixel(c_p.row + 1, c_p.col + 1); 
			break;
		case DOWN:
			p_new = pixel(c_p.row + 1, c_p.col); 
			break;
		case LEFT_DOWN:
			p_new = pixel(c_p.row + 1, c_p.col - 1); 
			break;
	}  


	explore_draw(map,p,p_new, dir, move, nodes); 

}

void roadmap::explore_draw(cv::Mat & map, const pixel p, const pixel c_p, const direction dir, const pixel end) const noexcept {

	if(c_p == end)
		return;	


	if (p != c_p) {  
		map.at<cv::Vec3b>(c_p.row, c_p.col) = cv::Vec3b(0,255,0);  	     	
	}

	pixel p_new(-1,-1);
	switch(dir) {
		case LEFT:
			p_new = pixel( c_p.row, c_p.col - 1); 
			break;
		case LEFT_UP:
			p_new = pixel(c_p.row - 1, c_p.col - 1); 
			break;
		case UP:
			p_new = pixel(c_p.row - 1, c_p.col); 
			break;
		case RIGHT_UP:
			p_new = pixel(c_p.row - 1, c_p.col + 1); 
			break;
		case RIGHT:
			p_new = pixel(c_p.row, c_p.col + 1); 
			break;
		case RIGHT_DOWN:
			p_new = pixel(c_p.row + 1, c_p.col + 1); 
			break;
		case DOWN:
			p_new = pixel(c_p.row + 1, c_p.col); 
			break;
		case LEFT_DOWN:
			p_new = pixel(c_p.row + 1, c_p.col - 1); 
			break;
	}  


	explore_draw(map,p,p_new, dir, end); 

} 


bool roadmap::move(std::vector<cv::Point> & n1, const cv::Mat & brushfire, cv::Mat & map, cv::Point start, cv::Point end) {


	cv::Point current = start;
	while(current != end) {

		// Find direction
		int y = end.y - start.y;
		int x = end.x - start.x;

		direction dir;

		if( y < 0) {

			if(x < 0) 
				dir = LEFT_UP;
			else
				dir = RIGHT_UP;

		} else if( y > 0) {

			if( x < 0) 
				dir = LEFT_DOWN;
			else
				dir = RIGHT_DOWN;
		} else if(y == 0) {

			if(x < 0)
				dir = LEFT;	
			else 
				dir = RIGHT;

		}  else if(x == 0) {

			if(y < 0)
				dir = UP;	
			else
				dir = DOWN;


		} else {

			//		std::cout << "ERROR" << std::endl;
			dir = LEFT;

		}  

		cv::Point p_new(0, 0);
		switch(dir) {
			case LEFT:
				p_new += cv::Point(-1, 0); 
				break;
			case LEFT_UP:
				p_new += cv::Point( - 1,  - 1); 
				break;
			case UP:
				p_new += cv::Point(0, -1); 
				break;
			case RIGHT_UP:
				p_new += cv::Point( 1, -1); 
				break;
			case RIGHT:
				p_new += cv::Point(1,0); 
				break;
			case RIGHT_DOWN:
				p_new += cv::Point(1,1); 
				break;
			case DOWN:
				p_new += cv::Point(0,1); 
				break;
			case LEFT_DOWN:
				p_new += cv::Point(-1,1); 
				break;
		} 

		if(p_new == cv::Point(0,0)) { 
			//		std::cout << "ERROR 1" << std::endl;  
			return false;
		}

		uchar current_brush = brushfire.at<cv::Vec3b>(current.y, current.x)[0];  

		current += p_new;

		uchar new_brush = brushfire.at<cv::Vec3b>(current.y, current.x)[0];  

		if(new_brush < current_brush) { 
			//				std::cout << "ERROR 2" << std::endl;  
			return false;
		}

		switch(dir) {
			case LEFT:
				break;
			case LEFT_UP:
				if(brushfire.at<cv::Vec3b>(current.y, current.x+1) == cv::Vec3b(0,0,0)  ||  brushfire.at<cv::Vec3b>(current.y + 1, current.x) == cv::Vec3b(0,0,0) ) { 
					//					std::cout << "ERROR 3" << std::endl;  
					return false;
				}
				break;
			case UP:
				break;
			case RIGHT_UP:
				if(brushfire.at<cv::Vec3b>(current.y, current.x-1) == cv::Vec3b(0,0,0)  ||  brushfire.at<cv::Vec3b>(current.y + 1, current.x) == cv::Vec3b(0,0,0) ) { 
					//					std::cout << "ERROR 4" << std::endl;  
					return false;
				}
				break;
			case RIGHT:
				break;
			case RIGHT_DOWN:
				if(brushfire.at<cv::Vec3b>(current.y, current.x-1) == cv::Vec3b(0,0,0)  ||  brushfire.at<cv::Vec3b>(current.y - 1, current.x) == cv::Vec3b(0,0,0) ) { 
					//					std::cout << "ERROR 5" << std::endl;  
					return false;
				}
				break;
			case DOWN:
				break;
			case LEFT_DOWN:
				if(brushfire.at<cv::Vec3b>(current.y, current.x+1) == cv::Vec3b(0,0,0)  ||  brushfire.at<cv::Vec3b>(current.y - 1, current.x) == cv::Vec3b(0,0,0) ) { 
					//					std::cout << "ERROR 6" << std::endl;  
					return false;
				}
				break;
		} 


		if(brushfire.at<cv::Vec3b>(current.y, current.x) == cv::Vec3b(0,0,0)) { 
			//			std::cout << "ERROR 7" << std::endl;  
			return false;
		}

		if((map.at<cv::Vec3b>(current.y, current.x) == cv::Vec3b(0,255,0) && map.at<cv::Vec3b>(current.y, current.x) == cv::Vec3b(255,0,0)) && current != end) { 
			//			std::cout << "ERROR 8" << std::endl;  
			return false;	
		}

		n1.push_back(current); 
	} 


	//	std::cout << "TRUE " << start << end << std::endl; 
	return true;
} 



void roadmap::move_draw(const std::vector<cv::Point> & n1, std::vector<pixel> & n2, cv::Mat & map){
	for(const cv::Point p : n1 )
	{
		if(map.at<cv::Vec3b>(p.y,p.x) != cv::Vec3b(0,255,0)  ) { 
			n2.push_back(pixel(p.y,p.x));
			map.at<cv::Vec3b>(p.y, p.x) = cv::Vec3b(0,255,0);  
		}
	}

}  


int roadmap::remove_duplicate_nodes(std::vector<pixel> & nodes) {

	int count = 0;
	for(size_t i = 0; i < nodes.size(); i++ )
	{
		for(size_t j = 0; j < nodes.size(); j++ )
		{

			if(i != j) {

				if(nodes[i] == nodes[j]) { 
					nodes.erase(nodes.begin() + j ); 
					count++;
				}
			}  
		}



	}


	return count;

}

void roadmap::enterMap(const float x, const float y, float &tx, float &ty, cv::Mat& world, cv::Mat& map) {
    float dx = 100, dy = 100;                   // Delta x,y in coords
    float px, py, clsX, clsY;                   // Both pairs are tempoaty target coords
    int index;
    cv::Point tgtPxl, robPxl;

    cv::waitKey(1);

    cv::Vec3b black{0, 0, 0}; // Black color
    bool wallHit = false;

    for (int i = 0; i < graph.size(); i++) {
        // Avoid end-nodes
        if ( graph[i].road_numbers.size() <= 1 ) {
            continue;
        }

        // Check for  collision with walls
        wallHit = false;
        tgtPxl.x = graph[i].point.col, tgtPxl.y = graph[i].point.row;
        robPxl.x = int( x / resizeFactor ) + int(roadMap.cols / 2);
        robPxl.y = -int( y / resizeFactor ) + int(roadMap.rows / 2);

        cv::LineIterator it(world, robPxl, tgtPxl);
        for (int j = 0; j < it.count; j++, ++it) {
            if ( world.at<cv::Vec3b>(it.pos()) == black ){
                wallHit = true;
                break;
            }
        }
        if (wallHit) {
            continue;
        }

        // Target intersection is approved
        px = ( float(tgtPxl.x) - float(roadMap.cols) / 2 ) * resizeFactor;
        py = -( float(tgtPxl.y) - float(roadMap.rows) / 2 ) * resizeFactor;

        if ( sqrt((px-x)*(px-x) + (py-y)*(py-y)) < sqrt((dx*dx)+(dy*dy)) ) {
            dx = abs(x - px); dy = abs(y - py);
            clsX = px; clsY = py;
            index = i;
        }
    }
    map.at<cv::Vec3b>(graph[index].point.row, graph[index].point.col) = cv::Vec3b{255,0,255};

    tx = clsX; ty = clsY;
}

int roadmap::enterPath(const float rX, const float rY, cv::Mat &world) {
    int dx = 100, dy = 100;
    cv::Point tPxl, rPxl;
    int nodeIndex;

    cv::Vec3b black{0, 0, 0};
    bool wallHit = false;

    for (int i = 0; i < graph.size(); i++) {
        /// Avoid end-nodes
        if ( graph[i].road_numbers.size() <= 1 ) {
            continue;
        }

        /// Check for collision with walls
        wallHit = false;
        tPxl.x = graph[i].point.col; tPxl.y = graph[i].point.row;
        rPxl.x = int( rX / resizeFactor ) + int(roadMap.cols / 2);
        rPxl.y = -int( rY / resizeFactor ) + int(roadMap.rows / 2);

        cv::LineIterator it(world, rPxl, tPxl);
        for (int j = 0; j < it.count; j++, ++it) {
            if ( world.at<cv::Vec3b>(it.pos()) == black ) {
                wallHit = true;
                break;
            }
        }
        if ( wallHit ) {
            continue;
        }

        /// Target node is valid, check if closest yet (manhattan distance |x1-x2|+|y1-y2|)
        if ( (abs(tPxl.x - rPxl.x) + abs(tPxl.y - rPxl.y) ) < ( dx + dy ) ) {
            dx = abs(tPxl.x - rPxl.x);
            dy = abs(tPxl.y - rPxl.y);
            nodeIndex = i;
        }
    }
    return nodeIndex;
}


int roadmap::planPath(const float rX, const float rY, const float goalX, const float goalY, cv::Mat& world, cv::Mat& map) {

    path.clear();
    pathCreated = false;
    goalCoords = std::tie(goalX, goalY);

    int startNode = enterPath(rX, rY, world);
    int endNode = enterPath(goalX, goalY, world);

    /// Based on Dijkstra's Weighted Shortest Path
    std::vector<int> nodeDist;
    std::vector<bool> nodeKnown;
    std::vector<int> previousNode;

    for ( int i = 0; i < graph.size(); i++ ) {
        nodeDist.push_back(std::numeric_limits<int>::max()); // Set distance to "infinity"
        nodeKnown.push_back(false);
        previousNode.push_back(-1);
    }

    nodeDist[startNode] = 0;

    std::cout << "\n";

   // std::cout << "Node\tKnown\tDist\tPrev\n";
    while( true ) {

        /// Find unknown node with smallest distance
        int crntNode = -1, smallDist = std::numeric_limits<int>::max();
        for (int i = 0; i < graph.size(); i++) {
            if (nodeKnown[i] == false) {
                if (nodeDist[i] < smallDist) {
                    smallDist = nodeDist[i];
                    crntNode = i;
                }
            }
        }
        /// If crntNode == -1 then there are no more nodes
        if (crntNode == -1) {
            break;
        }
        map.at<cv::Vec3b>(graph[crntNode].point.row, graph[crntNode].point.col) = {255,128,128};
        nodeKnown[crntNode] = true;

        /// For each node adjacent to crntNode
        for (int i = 0; i < graph[crntNode].road_numbers.size(); i++) {

            int thisRoad = graph[crntNode].road_numbers[i];
            int adjNode = -1;
            /// Avoid invalid roads
            if (roadsGraph[thisRoad].pt1 == -1 || roadsGraph[thisRoad].pt2 == -1) {
                std::cout << "invalid adjNode\n";
                continue;
            } else if (roadsGraph[thisRoad].pt1 == crntNode) {
                adjNode = roadsGraph[thisRoad].pt2;
            } else {
                adjNode = roadsGraph[thisRoad].pt1;
            }

            /// If the neighbor is unknown and the distance is bigger than the current + road:
            if ( nodeKnown[adjNode] == false ) {
                if (nodeDist[crntNode] + roadsGraph[thisRoad].length < nodeDist[adjNode]) {

                    nodeDist[adjNode] = nodeDist[crntNode] + roadsGraph[thisRoad].length;
                    previousNode[adjNode] = crntNode;
                }
            }

        }
    //std::cout << crntNode << "\t\t" << nodeKnown[crntNode] << "\t\t" << nodeDist[crntNode] << "\t\t" << previousNode[crntNode] << "\t\t(" << graph[crntNode].point.col << " , " << graph[crntNode].point.row << ")\n";
    }


    map.at<cv::Vec3b>(-int( rY / resizeFactor ) + int(roadMap.rows / 2), int( rX / resizeFactor ) + int(roadMap.cols / 2)) = {0,0,128};
    map.at<cv::Vec3b>(-int( goalY / resizeFactor ) + int(roadMap.rows / 2), int( goalX / resizeFactor ) + int(roadMap.cols / 2)) = {0,128,0};

    /// Assemble path from startNode to endNode:
    int nextNode = endNode;

    while(true){
        path.push_back(nextNode);

        if (previousNode[nextNode] == -1) {
            break;
        }
        else {
            nextNode = previousNode[nextNode];
        }
    }

    std::reverse(path.begin(), path.end() );
    pathCreated = true;

    return path.size();
}

void roadmap::navigate(int& pathProg, float &tgtX, float &tgtY, bool& goalReached, cv::Mat map) {
     if (pathCreated) {
        if (pathProg > -2 && pathProg < u_char(path.size() - 1)) {
            if (pathProg != -1) {
                map.at<cv::Vec3b>(graph[path[pathProg]].point.row, graph[path[pathProg]].point.col) = {0, 128, 255};
            }

            int nextNode = path[pathProg + 1];
            pathProg += 1;
            goalReached = false;
            tgtX = (float(graph[nextNode].point.col) - float(roadMap.cols) / 2) * resizeFactor;
            tgtY = -(float(graph[nextNode].point.row) - float(roadMap.rows) / 2) * resizeFactor;

            map.at<cv::Vec3b>(graph[nextNode].point.row, graph[nextNode].point.col) = {0, 0, 255};

        }
        else if (pathProg == u_char(path.size() - 1)) {
            map.at<cv::Vec3b>(graph[path[pathProg]].point.row, graph[path[pathProg]].point.col) = {0, 128, 255};

            tgtX = std::get<0>(goalCoords);
            tgtY = std::get<1>(goalCoords);
            pathProg += 1;
        }
        else if (pathProg > u_char(path.size() - 1)) {
            goalReached = true;
            tgtX = std::numeric_limits<float>::max();
            tgtY = std::numeric_limits<float>::max();
            map.at<cv::Vec3b>(graph[path[pathProg+1]].point.row, graph[path[pathProg+1]].point.col) = {0, 128, 255};

        }
        else {
            pathProg = -2;
            tgtX = std::numeric_limits<float>::max();
            tgtY = std::numeric_limits<float>::max();
        }
     }
     else {
         pathProg = -2;
         tgtX = std::numeric_limits<float>::max();
         tgtY = std::numeric_limits<float>::max();
     }
}

void roadmap::draw_world_overlay(cv::Mat &roadmap, cv::Mat &world) {
    cv::Vec3b white{255,255,255};
    for (int i = 0; i < roadmap.rows; i++) {
        for (int j = 0; j < roadmap.cols; j++) {
            if (roadmap.at<cv::Vec3b>(i,j) == white && world.at<cv::Vec3b>(i,j) != white) {
                roadmap.at<cv::Vec3b>(i,j) = cv::Vec3b{0, 0, 0};
            }
        }
    }
}

void roadmap::drawPath(cv::Mat map) {
    if (pathCreated){
        int x, y;
        for ( int i = 0; i < path.size(); i++ ) {
            x = graph[path[i]].point.col;
            y = graph[path[i]].point.row;
            map.at<cv::Vec3b>(y, x) = {255, 0, 255};
        }
    }
}





