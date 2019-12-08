#pragma once

#include<cmath>
#include<iostream>


class vec2 {

	public:
		vec2(); 
		vec2(const double xx, const double yy); 
		double length() const; 
		
		double getX() const;
	        double getY() const; 	

		void normalize(); 

		vec2 add(const vec2 b) const;
	        vec2 sub(const vec2 b) const;
	        double cross(const vec2 b) const;
		double dot(const vec2 b) const; 
	        vec2 scalar(const double b) const; 	

		~vec2(); 
		
		vec2 operator+(const vec2 b) const;
	        vec2 operator-(const vec2 b) const; 	
		double operator*(const vec2 b) const;
	        vec2 operator*(const double b) const;  	

	private: 
		double x;
		double y;

		friend std::ostream & operator<<(std::ostream & os, vec2 & v); 

}; 


std::ostream & operator<<(std::ostream & os, vec2 & v);
