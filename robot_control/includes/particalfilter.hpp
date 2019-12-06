#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <functional> 
#include <exception>
#include <string>
#include <random>

#define DEBUG_PARTICLEFILTER 2
#define SAVE_DATA 1
#define VERBOSE 1

class Empty : std::exception {

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

class ParticleFilter {

	public:
		// Init x and y position and heading
		ParticleFilter(const int offset_x, const int offset_y); 
		ParticleFilter(const int offset_x, const int offset_y, const int x, const int y, const float heading, const float std[]); 
		void generate_lookup_table(const cv::Mat & map, const double MeterPrPixel, const double max_meter, const double resolutation_angle, const int number_of_threads = 2); 
		void load_lookup_table(const std::vector<cv::String> filenames, const cv::Mat & map, const int res, const int number_of_threads = 2); 
		void init(const int x, const int y, const float theta, const float std[]); 
		void prediction(const float delta_t, const double std_pos[], const double velocity, const double yaw_rate); 

		std::tuple<const double, const double, const double> dataAssociation(std::vector<std::tuple<double, double>> lidar_data, double sigma, const double res);

		void resample();

		void draw_particles(cv::Mat & map, const double res); 
		void draw_robot_pos(cv::Mat & map, const double x, const double y, const double res); 
		void draw_estime_pos(cv::Mat & map, const double x, const double y, const double res); 
		void resize(cv::Mat & src, cv::Mat & dst, const double MeterPrPixel);


		~ParticleFilter(); 

		void generate_lookup_table2(const cv::Mat & map, const double MeterPrPixel, const double max_meter, const double resolutation_angle, const int i_index_start, const int i_index_end); 

		void load_lookup_table2(const std::string filenames, const int res); 
	private:

		
		const int offset_x;
		const int offset_y;

		int num_particles = 200;
		struct particle {

			int id; // Index
			double x;   // current x coordinate
			double y;   // current y coordinate
			double theta;// Heading
			float s;   // scale
			int x0;  // original x coordinate
			int y0;  // original y coordinate
			float theta0; // Heading
			double w;   // weight


			void change_theta(double x) {
				if(x < 0) 
					theta = x + 360;
				else if( x >= 360) 
					theta = x - 360;
				else
					theta = x;
			
			
			}  
		};


		struct lines {
			std::vector<double> distance; 	
			bool used = false;
		}; 


		std::vector<std::vector<lines>> lookup;
		std::vector<particle> particles;

		std::mt19937 gen;	
		//	double sum(const std::vector<double> x, const int begin, const int end, double (* expression)(double) );
		double sum(const std::vector<double> x, const int begin, const int end, const std::function<double (double)> expression  );

		double maximum_likelihood(const double mu, const double sigma, const std::vector<double> x, const int n, const int start = 0); 
		
		double maximum_likelihood(const double mu, const double sigma, const double x); 
}; 
