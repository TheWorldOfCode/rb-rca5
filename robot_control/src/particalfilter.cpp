
#include "../includes/particalfilter.hpp" 

#include <algorithm>
#include <iostream>
#include <numeric>
#include <thread>
#include <fstream>

using namespace std;

ParticleFilter::ParticleFilter(const int offset_x, const int offset_y) : offset_x(offset_x), offset_y(offset_y), gen(mt19937(std::random_device{}()  ) )    { }  

ParticleFilter::ParticleFilter(const int offset_x, const int offset_y, const int x, const int y, const float theta, const float std[]) : offset_x(offset_x), offset_y(offset_y)   {
	init(x,y,theta, std); 
}  

void ParticleFilter::generate_lookup_table(const cv::Mat & map, const double MeterPrPixel, const double max_meter, const double resolutation_angle, const int number_of_threads ) {
#if VERBOSE == 1
	std::cout << "lookup table generating" << std::endl; 
#endif 
	lookup.resize(map.cols); 

	if(number_of_threads > 1) { 

		vector<thread> threads;
		threads.resize(number_of_threads); 

		int i = 0; 
		int res = (map.cols - 1) / number_of_threads;
		int e = res;
		for(int t = 0; t < number_of_threads ; t++ , i += res, e += res  )
		{ 
			if((size_t) e > lookup.size() || t == number_of_threads - 1) { 
				e = lookup.size();
			}

			threads[t] = thread(&ParticleFilter::generate_lookup_table2, this , map, MeterPrPixel, max_meter, resolutation_angle, i, e);
		}

		for(size_t t = 0; t < threads.size(); t++ ) { 
			threads[t].join();  
			std::cout << "Joining thread" << std::endl; 
		}
	} else
		generate_lookup_table2(map, MeterPrPixel, max_meter, resolutation_angle, 0, lookup.size() ); 

#if VERBOSE == 1
	std::cout << "lookup table generated" << std::endl; 
#endif
} 

void ParticleFilter::load_lookup_table(vector<cv::String> filenames, const cv::Mat & map, const int res, const int number_of_threads) {

	lookup.resize(map.cols); 

	for(size_t i  = 0;  i < lookup.size(); i++ )
	{
	  lookup[i].resize(map.rows);  
	}
	
		
	
	vector<thread> threads;
	threads.resize(number_of_threads); 
	size_t thread_n = -1;

	for(std::string f : filenames )
	{
		std::cout << "Loading lookup tabel from " << f << std::endl; 
		threads[++thread_n] = thread(&ParticleFilter::load_lookup_table2, this,f,res); 

		if(thread_n == threads.size() - 1)  {
			for(size_t i = 0; i < threads.size(); i++)
				threads[i].join();  
			thread_n = -1;
		} 
	}


	if((int) thread_n != -1) {
			for(size_t i = 0; i < thread_n + 1 ; i++)
				threads[i].join();  
			thread_n = -1;
	}  


}  


void ParticleFilter::init(const int x, const int y, const float theta, const float std[]) {
#if VERBOSE == 1
	std::cout << "Init" << std::endl; 
#endif
	particles.resize(num_particles); 

	float std_x, std_y, std_theta;

	std_x = std[0];
	std_y = std[1];
	std_theta = std[2];



	normal_distribution<float> dist_x(x, std_x); 
	normal_distribution<float> dist_y(y, std_y); 
	normal_distribution<float> dist_theta(theta, std_theta); 

	for(int i = 0; i < num_particles; i++ )
	{
		particle p;

		p.id = i;
		p.x = dist_x(gen); 
		p.y = dist_y(gen); 
		p.theta = dist_theta(gen); 
		p.w = 1 /((double) num_particles);

		particles[i] = p;
	}

#if VERBOSE == 1
	std::cout << "Init done" << std::endl; 
#endif

}  

void ParticleFilter::prediction(const float delta_t, const double std_pos[], const double velocity, const double yaw_rate){
#if DEBUG_PARTICLEFILTER == 1
	std::cout << "Entering prediction" << std::endl;  
#endif

#if SAVE_DATA == 1
	static int index = 0;
	ofstream out;

	out.open("../test/localization/dataprediction" + to_string(index++) + "Speed" + to_string(velocity)   + ".txt"); 

	out << "{" << std::endl;
	out << "\tsigma: " << std_pos[0] << " " << std_pos[1] << " "  << std_pos[2] << std::endl;   	
	out << "\tdelta: " << delta_t << std::endl;
	out << "\tVelocity: " << velocity << std::endl;
	out << "\tyaw_rate: " << yaw_rate << std::endl;
	out << "}" << std::endl; 

#endif 

	double std_x, std_y, std_theta;
	std_x = std_pos[0];
	std_y = std_pos[1];
	std_theta = std_pos[2];

	for(int i = 0; i < num_particles ; i++ )
	{
		particle * p = &particles[i];

		const double new_x = p->x + velocity * delta_t * cos(p->theta * M_PI/((double) 180) + yaw_rate * delta_t);   
		const double new_y = p->y - velocity * delta_t * sin(p->theta * M_PI/((double) 180) + yaw_rate * delta_t);   
		const double new_theta = p->theta + yaw_rate * delta_t * ((double)  180)/M_PI;

		       	
		normal_distribution<double> dist_x(new_x, std_x); 
		normal_distribution<double> dist_y(new_y, std_y); 
		normal_distribution<double> dist_theta(new_theta, std_theta); 

#if SAVE_DATA == 1 

		out << "{" <<std::endl; 
		out << "\tx: " << p->x << std::endl; 
		out << "\ty: " << p->y << std::endl; 
		out << "\ttheta: " << p->theta << std::endl; 
		out << "\tx_new: " << new_x << std::endl; 
		out << "\ty_new: " << new_y << std::endl; 
		out << "\ttheta_new: " << new_theta << std::endl; 
#endif 

		const double x_std = dist_x(gen);
		const double y_std = dist_y(gen);  
		const double theta_std = dist_theta(gen); 

		p->x = x_std;
		p->y = y_std;
		p->change_theta(theta_std); 

#if SAVE_DATA == 1
		out << "\tx_std: " << x_std << std::endl; 
		out << "\ty_std: " << y_std << std::endl; 
		out << "\ttheta_std: " << theta_std << std::endl; 
		out << "}" << std::endl;
#endif
	}

#if DEBUG_PARTICLEFILTER == 1
	std::cout << "Exiting prediction" << std::endl;  
#endif
}  

std::tuple<const double, const double, const double> ParticleFilter::dataAssociation(std::vector<std::tuple<double, double>> lidar_data, double sigma, const double res) {
#if DEBUG_PARTICLEFILTER == 1
	std::cout << "Entering data Association" << std::endl; 
#endif

#if SAVE_DATA == 1
	static int index = 0;
	ofstream out;
	out.open("../test/localization/dataAssociation" + to_string(index++) + ".txt"); 

#endif

	double eta = 0;

	if(lidar_data.size() == 0 ) 
		throw Empty("Lidar data", "ParticleFilter::dataAssociation", 141); 


#if SAVE_DATA == 1
	out << "{ \n" << "lidar data" << std::endl; 	

	for(const std::tuple<double, double> d : lidar_data )
		out << "\t" << std::get<0>(d) << " " << std::get<1>(d) << std::endl; 

	out << "}" << std::endl; 

#endif

	for(size_t i = 0; i < particles.size(); i++ )
	{
		particle * p = &particles[i];

		const int histX = round(p->x/res) + offset_x;
	        const int histY = round(p->y/res) + offset_y;

		const vector<double> histogram  = lookup[histX][histY].distance;

		if(histogram.size() == 0 && lookup[histX][histY].used)
			throw Empty("Histogram particle number " + to_string(i) + " (x,y):  (" + to_string(histX) + ", " + to_string(histY) + ")", "ParticleFilter::dataAssociation", 150);

		if(lookup[histX][histY].used) { 

			const double res = 360 / histogram.size(); 


			double w = 1;
			for(const tuple<double, double> d : lidar_data )
			{

				int histIndex = round((get<0>(d)  + p->theta )/res);

				if(histIndex < 0)
					histIndex += 360;
				else if(histIndex > 359) 
					histIndex -= 360;

				//			w += maximum_likelihood(histogram[histIndex], sigma, lidar_distance, lidar_distance.size()); 
				w += maximum_likelihood(histogram[histIndex], sigma, get<1>(d)); 
			}




			eta += w;

#if SAVE_DATA == 1
			out << "{" << std::endl;
			out << "\t index: " << i << " of " << num_particles << std::endl;   
			out << "\t id: " << p->id << std::endl; 	
			out << "\t x: " << p->x << std::endl; 	
			out << "\t y: " << p->y << std::endl; 	
			out << "\t x + offset: " << offset_x << std::endl; 	
			out << "\t y + offset: " << offset_y << std::endl; 	
			out << "\t hist x: " << histX << std::endl; 	
			out << "\t hist y: " << histY << std::endl; 	
			out << "\t Theta: " << p->theta << std::endl; 
			out << "\t Resolutation: " << res << std::endl;  
			out << "\t sigma: " << sigma << std::endl; 
			out << "\t Old weight: " << p->w << std::endl; 
			out << "\t New weight: " << w << std::endl; 
			out << "}" << std::endl;
#endif

			p->w = w;
		} else
			p->w = 0;
	}

#if SAVE_DATA == 1
	out << "{" << std::endl;
	out << "\t eta: " << eta << std::endl; 
	out << "}" << std::endl;
#endif

	for(size_t i = 0; i < particles.size(); i++ )
	{
		particle * p = &particles[i];
		p->w *= 1/eta;
	}


	double x = 0;
	double y = 0;	
	double theta = 0;
	// Estime position
	for(size_t i = 0; i < particles.size(); i++ )
	{
		particle * p = &particles[i];

		x +=  p->x * p->w;
		y += p->y * p->w;
		theta += p->theta * p->w;
	}


	//	std::cout << x << " " << y << " " << theta << std::endl;  


#if SAVE_DATA == 1
	out.close(); 
#endif


	resample(); 

#if DEBUG_PARTICLEFILTER == 1
	std::cout << "Exiting data Association" << std::endl; 
#endif

	return std::tuple<const double, const double, const double>(x,y,theta); 
} 

double doubleRandom(double a, double b) {
	double random = ((double) rand()) / (double) RAND_MAX;
	double diff = b - a;
	double r = random * diff;
	return a + r;
}  

void ParticleFilter::resample() {


	vector<particle> M;
	//double delta = doubleRandom(0, 1/((double) num_particles ) );
	uniform_real_distribution<double> dis(0, 1/((double) num_particles ) );
	double delta = dis(gen);  
	double c = particles[0].w;	

	int i = 0;

	for(int j = 0; j < num_particles ; j++ )
	{
		double u = delta + j * 1/((double) num_particles);  

		while(u > c) {

			i += 1;
			c += particles[i].w;

		}  

		particle p = particles[i];
		p.w = 1/((double) num_particles);
		// Create new particle
		M.push_back(p); 
	}


	particles = M;

	num_particles = particles.size();
	/*
	   vector<particle> copy = particles;

	   particles.clear();

	   vector<double> weights;

	   for(const particle p : copy )
	   weights.push_back(p.w);  


	   std::discrete_distribution<int> weights_dist(weights.begin(), weights.end()); 


	   for(int i = 0; i < num_particles; i++ )
	   {
	   int index = weights_dist(gen);
	   particles.push_back(copy[index] );  
	   } */

}  

void ParticleFilter::draw_particles(cv::Mat & map, const double res) {
#if DEBUG_PARTICLEFILTER == 1
	std::cout << "Drawing particles" << std::endl;
#endif

	for(const particle p : particles ) { 
		int x = round(p.x/res);
		int y = round(p.y/res);
		//	map.at<cv::Vec3b>(y + map.rows/2, x + map.cols/2) = cv::Vec3b(0, 255/2, 255/2);  
		cv::circle(map, cv::Point(x + map.cols/2, -y + map.rows/2), 2, cv::Vec3b(0,255/2,255/2 )); 
	}

#if DEBUG_PARTICLEFILTER == 1
	std::cout << "Drawing particles done" << std::endl;
#endif

}  

void ParticleFilter::draw_robot_pos(cv::Mat & map, const double x, const double y, const double res) {


#if DEBUG_PARTICLEFILTER == 1
	std::cout << "Drawing Robot position" << std::endl;
#endif
	int x1 = round(x/res);
	int y1 = round(y/res);
	cv::circle(map, cv::Point(x1 + map.cols/2, y1 + map.rows/2), 4, cv::Vec3b(255,0, 0 )); 

#if DEBUG_PARTICLEFILTER == 1
	std::cout << "Drawing Robot position done" << std::endl;
#endif

}  

void ParticleFilter::draw_estime_pos(cv::Mat & map, const double x, const double y, const double res) {


#if DEBUG_PARTICLEFILTER == 1
	std::cout << "Drawing estimeret position" << std::endl;
#endif
	int x1 = round(x/res);
	int y1 = round(y/res);
	cv::circle(map, cv::Point(x1 + map.cols/2, -y1 + map.rows/2), 4, cv::Vec3b(0,255,0)); 

#if DEBUG_PARTICLEFILTER == 1
	std::cout << "Drawing estimerte position done" << std::endl;
#endif

}  


void ParticleFilter::resize(cv::Mat & src, cv::Mat & dst, const double MeterPrPixel) {

	cv::resize(src,dst, cv::Size()  , 1/1.41735 * 1/MeterPrPixel , 1/1.41735 * 1/MeterPrPixel, cv::INTER_CUBIC); 

}  

ParticleFilter::~ParticleFilter() {} 


void ParticleFilter::generate_lookup_table2(const cv::Mat & map, const double MeterPrPixel, const double max_meter, const double resolutation_angle, const int i_index_start, const int i_index_end ) {

#if SAVE_DATA == 1
	std::ofstream out;
	out.open("../test/localization/" + to_string(i_index_start) + ".txt"); 
	out << "Start " << i_index_start << " End " << i_index_end << std::endl;   
	out << "Resoluation " << resolutation_angle << " MeterPrPixel " << MeterPrPixel << " max meter " << max_meter << std::endl;   
#endif

	const int rows = map.rows;
	const int cols = map.cols;
	for(size_t i = i_index_start; (int) i < i_index_end; i++ )
	{
		lookup[i].resize(map.rows);  
		for(size_t j = 0; j < lookup[i].size(); j++ )
		{

			if(!(map.at<cv::Vec3b>(j,i) == cv::Vec3b(0,0,0))) { 

#if SAVE_DATA == 1
				out << "{" << std::endl;
				out << " col " << i << " row " << j << std::endl;   	
#endif
				lines l;
				for(float angle = 0; angle < 360; angle+=resolutation_angle  )
				{

					cv::Point p1(i,j);  
					cv::Point p2(i + cos(angle * M_PI/((double) 180 ) ) * max_meter/MeterPrPixel * 1.2, j - sin(angle* M_PI/((double) 180 ) ) * max_meter/MeterPrPixel * 1.2 );

					cv::LineIterator itr(map, p1, p2 ); 
					int count = 0;


					cv::Point tmp2; 
					while( map.at<cv::Vec3b>(itr.pos().y, itr.pos().x ) == cv::Vec3b(255,255,255) && itr.count != count  ) { 
						tmp2 = cv::Point(itr.pos().x, itr.pos().y) ;

						++count;
						++itr;
						const cv::Point tmp(itr.pos().x, itr.pos().y) ; 
						const int x = tmp.x;
						const int y = tmp.y;

						if(cols - x > 0 && x < 0 ) 
							break;

						if(y < 0 && rows-y > 0) 
							break;
					}



					double meter = min(sqrt( (tmp2.x - p1.x) * (tmp2.x - p1.x) + (tmp2.y -p1.y) * (tmp2.y -p1.y) )* MeterPrPixel, max_meter);

#if SAVE_DATA == 1
					out << "\t"  << angle << " " << meter << " " << count << " " << itr.count   << std::endl; 
#endif


					l.distance.push_back(meter); 

				}

				l.used = true;

#if SAVE_DATA == 1
				out << "}" << std::endl;
#endif

				lookup[i][j] = l;

			}
		}
	}

#if SAVE_DATA == 1
	out << "Done"; 
	out.close(); 
#endif

} 


void ParticleFilter::load_lookup_table2(const std::string filename, const int res) 
{

	ifstream in;
	in.open(filename); 

	char line[255];

	int x;
	int y;
	while(!in.eof()) { 
		in.getline(line,255); 

		if(line[0] == '{' ) {
			in.getline(line,255, ' '); 
			in.getline(line,255, ' '); 
			in.getline(line,255, ' '); 
			x = atoi(line); 
			in.getline(line,255, ' '); 
			in.getline(line,255); 
			y = atoi(line); 

			lines dis;

			in.getline(line,255); 
			while(line[0] != '}'  ) {

				int count = 0;
				char line2[res];
				int j = 0;
				for(int i = 0; i < 255 ; i++ )
				{
					if(line[i] == ' ')
						count++;
					else if(count == 1) { 
						line2[j++] = line[i]; 
					}
					if(count > 1) 
						break;

				}

				dis.distance.push_back(atof(line2));  
				in.getline(line,255); 
			} 
			dis.used = true;

			lookup[x][y] = dis;



		} 

	}

	in.close();   

} 

double ParticleFilter::sum(const vector<double> x, const int begin, const int end, const function<double (double)> expression ) {

	int sum = 0;
	for(int i = begin; i < end ; i++ )
	{
		sum += expression(x[i]);  
	}


	return sum;

}  


double ParticleFilter::maximum_likelihood(const double mu, const double sigma, const std::vector<double> data, const int n, const int start) {

	auto sumExpression = [mu](double x) -> double{ return (x - mu) * (x - mu); };

	return std::pow( 1 / (2 * M_PI * sigma*sigma), n/2) * std::exp(- (sum(data,start,n, sumExpression ))/(2 * sigma * sigma));

}  



double ParticleFilter::maximum_likelihood(const double mu, const double sigma, const double x) {

	auto sumExpression = [mu](double x) -> double{ return (mu -x ) * ( mu - x); };

	return  1 / (sqrt(2 * M_PI)  * sigma) * std::exp(- (sumExpression(x))/(2 * sigma * sigma));

}  
