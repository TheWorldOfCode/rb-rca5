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
	point = vec2(x1, y1); 	
	endPoint = vec2(x2,y2); 



}  

double line::distance_to_point(vec2 p) {


	double num = (p - point) * direction;

	return num / direction.length()  ;

}  


double line::distance_to_point(double r, double theta) {

	vec2 P0( r*cos(theta) ,  r*sin(theta) ); 	

	double num = (P0 - point)* direction;

	return num / direction.length()  ;

}  

double line::distance_to_point(std::tuple<double,double> p) {

	return distance_to_point(get<0>(p), get<1>(p));
}  


void line::get_point_to_draw_the_line(vec2 & startpt, vec2 & endpt) const {

	startpt = point;
	endpt = endPoint;
} 


line::~line() {}

ostream & operator<<(ostream & os, line & l) {

	return os << l.point << " + t * " << l.direction;    

}  
