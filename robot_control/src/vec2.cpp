
#include "../includes/vec2.hpp"

using namespace std;

vec2::vec2() {}  
vec2::vec2(const double xx, const double yy) : x(xx), y(yy)   {}  

double vec2::length() const { return sqrt( x * x + y * y); }  

double vec2::getX() const { return x; }
double vec2::getY() const { return y; }  

void vec2::normalize() {

	double l = length(); 

	x /= l;
	y /= l;

}  


vec2 vec2::add(const vec2 b) const { return vec2(x + b.getX() , y + b.getY() ); }  
vec2 vec2::sub(const vec2 b) const { return vec2(x-b.getX(), y-b.getY()); }  
double vec2::cross(const vec2 b) const { return ( x * b.getY() - y * b.getX() ); }  
double vec2::dot(const vec2 b) const {return x*b.getX() + y * b.getY(); }  
vec2 vec2::scalar(const double b) const { return vec2( x * b , y * b ); }  
 

vec2::~vec2() {} 


vec2 vec2::operator+(const vec2 b) const { return add(b); }  
vec2 vec2::operator-(const vec2 b) const { return sub(b); }
double vec2::operator*(const vec2 b) const { return dot(b); }
vec2 vec2::operator*(const double b) const {return scalar(b); }  

ostream & operator<<(ostream & os, vec2 & v) {

	return os << "( " << v.x << " , " << v.y << " )";   

}  
