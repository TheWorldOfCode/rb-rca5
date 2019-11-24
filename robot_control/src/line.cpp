#include "../includes/line.hpp"
#include <string>

using namespace std;

line::line() {}

line::line(double x1, double y1, double x2, double y2, pointType coordType) {

	double x1New, y1New, x2New, y2New;

	switch(coordType) {  
		case pointType::POLAR: 
			x1New = x1*cos(y1); 
			y1New = x1*sin(y1);

			x2New = x2*cos(y2); 
			y2New = x2*sin(y2); 
			break;
		case pointType::CARTESIAN:
			x1New = x1;
			y1New = y1;

			x2New = x2;
			y2New = y2;
			break;
	}

	direction = vec2(x2-x1, y2-y1);
	direction.normalize();  
        point = vec2(x1, y1); 	
	endPoint = vec2(x2,y2); 

}  

line::line(tuple<double, double> p1, tuple<double, double> p2, pointType coordType) {

	double pi1 = get<0>(p1);
        double theta1 = get<1>(p1); 	

	double pi2 = get<0>(p2);
        double theta2 = get<1>(p2); 	


	double x1, y1, x2, y2;
	switch(coordType) {  
		case pointType::POLAR:
			x1 = pi1*cos(theta1); 
			y1 = pi1*sin(theta1);

			x2 = pi2*cos(theta2); 
			y2 = pi2*sin(theta2); 
			break;
		case pointType::CARTESIAN:
			x1 = pi1;
			y1 = theta1;

			x2 = pi2;
			y2 = theta2;
			break;
	}

	direction = vec2(x2-x1, y2-y1);
	direction.normalize(); 
	point = vec2(x1, y1); 	
	endPoint = vec2(x2,y2); 



}  

line::line(vec2 p1, vec2 p2, pointType coordType) {


	switch(coordType) {

		case pointType::POLAR:
			double x1, y1, x2, y2;

			x1 = p1.getX() * cos(p1.getY() );
		        y1 = p1.getX() * sin(p1.getY() );  	

			x2 = p2.getX() * cos(p1.getY() ); 
			y2 = p2.getX() * sin(p1.getY() ); 
			p1 = vec2(x1,y2);
		        p2 = vec2(x1,y2); 	
			break;
		case pointType::CARTESIAN:
			break;
	}  

	direction = p2 - p1;
	direction.normalize();
        
	point = p1; 
	endPoint = p2;
}  

vec2 line::getDirection() { return direction; }  

double line::distance_to_point(const vec2 p) const {


	double num = ((vec2)(p - point)).cross(direction);

	return num / direction.length()  ;

}  


double line::distance_to_point(const double r, const double theta) const {

	vec2 P0( r*cos(theta) ,  r*sin(theta) ); 	
	double num = ((vec2) (P0 - point)).cross(direction);



	return num / direction.length()  ;

}  

double line::distance_to_point( const std::tuple<double,double> p)  const{

	return distance_to_point(get<0>(p), get<1>(p));
}  

double line::distance_to_line(const line & l) const {

	throw "NOT IMPLEMENT"; 

}  


bool line::is_orgotonalt_to(const line & l) const { return direction * l.direction == 0 ? true : false; }  

bool line::is_orgotonalt_to(const line & l, const double threshold) const { 

	double dot = direction * l.direction;
	dot *= -1;

//	std::cout << "The dot product " << dot << " length "  << direction.length() << " "  << l.direction.length() << " Angle " << std::acos(dot)     << std::endl; 
	return 0-threshold < dot && dot < threshold ? true : false;

}  

bool line::is_orgotonalt_to(const line & l, const double lowerThreshold, const double upperThreshold) const { 

	double dot = direction * l.direction;

	dot *= -1;
	return lowerThreshold < dot && dot < upperThreshold ? true : false;

}  

void line::get_point_to_draw_the_line(vec2 & startpt, vec2 & endpt) const {

	startpt = point;
	endpt = endPoint;
} 


line::~line() {}

ostream & operator<<(ostream & os, line & l) {

	return os << l.point << " + t * " << l.direction;    

}  
