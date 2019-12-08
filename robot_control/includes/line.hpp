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
		line(vec2 p1, vec2 p2, pointType coordType = pointType::POLAR); 

		vec2 getDirection(); 
		
		double distance_to_point(const vec2 p) const;
		double distance_to_point(const double r, const double theta) const;
		double distance_to_point(const std::tuple<double,double> p) const; 
		double distance_to_line(const line & l) const;

		bool is_orgotonalt_to(const line & l) const;  
		bool is_orgotonalt_to(const line & l, const double threshold) const;  
		bool is_orgotonalt_to(const line & l, const double lowerThreshold, const double upperThreshold) const;  

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
