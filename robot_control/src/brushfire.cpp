
#include <vector>

#if DEBUG_BRUSHFIRE == 1
#include <iostream> 
#endif 

#include "../includes/brushfire.hpp" 

struct pixel_t {

	int row;
	int col;

}; 

void check_neighbors(cv::Mat & src, std::vector<pixel_t> & neighbors, int row, int col) {

	for(int r = row - 1; r < row + 2; r++ )
	{
		if(r > -1 && r < src.rows) {  

			for(int c = col - 1; c < col + 2 ; c++ )
			{
				if(c > -1 && c < src.cols) {
				
					if(src.at<cv::Vec3b>(r,c) == cv::Vec3b(255,255,255)) {
						pixel_t tmp;
						tmp.row = r;
						tmp.col = c;

						neighbors.push_back(tmp); 
					
					} 
				
				}  

			}
		}
	}


}  

int generate_brushfire(cv::Mat & source, cv::Mat & dst) {

#if DEBUG_BRUSHFIRE == 1 
	std::cout << "generating brushfire" << std::endl; 
#endif


	std::vector<pixel_t> neighbors;

	int rows = source.rows;
	int cols = source.cols;
	for(int row = 0; row < rows ; row++ )
	{
		for(int col = 0; col < cols ; col++ )
		{
			if(source.at<cv::Vec3b>(row,col) == cv::Vec3b(0,0,0)) 
				check_neighbors(source, neighbors, row, col); 

		}

	}

#if DEBUG_BRUSHFIRE == 1
	std::cout << "Checked the image for obstalce found " << neighbors.size() << " neighbors" << std::endl;   
#endif
	uchar color = BRUSHFIRE_BEGIN;

	dst = source.clone(); 
#if DEBUG_BRUSHFIRE == 1
	int test = 0;
	int test2 = 0;
#endif 
	while(neighbors.size() != 0 ) {
	
		std::vector<pixel_t> newneighbors;

		for(pixel_t pixel : neighbors )
		{

#if DEBUG_BRUSHFIRE == 1
			test2++;
#endif

		   if(dst.at<cv::Vec3b>(pixel.row, pixel.col) == cv::Vec3b(255,255,255) ) {

#if DEBUG_BRUSHFIRE == 1
			  test++;
#endif
		   
			   dst.at<cv::Vec3b>(pixel.row, pixel.col) = cv::Vec3b(color, color,color);  
			   check_neighbors(dst, newneighbors, pixel.row, pixel.col); 
		   
			   
		   }  
		}

		color += BRUSHFIRE_STEP_SIZE;
		neighbors = newneighbors;

#if DEBUG_BRUSHFIRE == 1
	std::cout << "There are " << newneighbors.size() << " new neighbors" << std::endl;  
#endif
	}  

	color -= BRUSHFIRE_STEP_SIZE;
	color -= BRUSHFIRE_STEP_SIZE;
#if DEBUG_BRUSHFIRE == 1
	std::cout << "Number of pixel painted " << test  << " of " << source.rows * source.cols << std::endl; 
	std::cout << "There are visited " << test2 << " pixels" << std::endl;  
	std::cout << "Brushfire generated (number of colors " << (int) color << ") " << std::endl; 
#endif

	return (int)color;
}
