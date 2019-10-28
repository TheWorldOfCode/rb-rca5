#pragma once

#include <iostream>
#include <string>
#include <tuple>

#include "../includes/vec2.hpp" 

enum pointType { POLAR, CARTESIAN};

class line {

	public: 
		line();
		line(double x1, double y1, double x2, double y2, pointType coordType = pointType::POLAR);
		line(std::tuple<double,double> p1, std::tuple<double,double> p2, pointType coordType = pointType::POLAR);
		
		double distance_to_point(vec2 p);
		double distance_to_point(double r, double theta);
		double distance_to_point(std::tuple<double,double> p); 

		void get_point_to_draw_the_line(vec2 & startpt, vec2 & endpt) const; 

		std::string print(); 
		~line(); 	



	private:
		vec2 direction;
		vec2 point;
		vec2 endPoint;

		friend std::ostream & operator<<(std::ostream & os, line & l); 
}; 


std::ostream & operator<<(std::ostream& os, line & l); 
