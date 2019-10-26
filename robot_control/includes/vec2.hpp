#pragma once

#include<cmath>
#include<iostream>


class vec2 {

	public:
		vec2(); 
		vec2(const double xx, const double yy); 
		double length(); 
		
		double getX() const;
	        double getY() const; 	


		vec2 add(const vec2 b) const;
	        vec2 sub(const vec2 b) const;
	        double cross(const vec2 b) const;
	        vec2 scalar(const double b) const; 	

		~vec2(); 
		
		vec2 operator+(const vec2 b);
	        vec2 operator-(const vec2 b); 	
		double operator*(const vec2 b);
	        vec2 operator*(const double b);  	

	private: 
		double x;
		double y;

		friend std::ostream & operator<<(std::ostream & os, vec2 & v); 

}; 


std::ostream & operator<<(std::ostream & os, vec2 & v);
