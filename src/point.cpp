#include <iostream>

#include <math.h>
#include "point.h"
#include "helpers.h"

Point::Point()
{

	std::cout << "Point Contructor void" << std::endl;
}

Point::Point(double x, double y)
{
	this->x = x;
	this->y = y;

}

Point::Point(double x, double y, double s, double d)
{
	this->x  = x;
	this->y  = y;
	this->s  = s;
	this->d  = d;

}

Point::Point(double x, double y, double s, double d, double yaw_deg, double speed)
{
	this->x  = x;
	this->y  = y;
	this->s  = s;
	this->d  = d;
	this->yaw_deg = yaw_deg;
	this->yaw_rad = deg2rad(yaw_deg);

	this->speed = speed;

}

// Point::Point(const Point &point)
// {
// 	*this = point;

// }

Point::~Point() 
{

}


void Point::print(void)
{
	std::cout << "Point (x: " << this->x << ", y: " << this->y << ", s: " << this->s << ", d: " << this->d  << ", yaw_deg: " << this->yaw_deg << std::endl;

}

double Point::distance(Point p)
{
	return distance(*this, p);
}


double Point::distance(Point p1, Point p2)
{
	return sqrt( (p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y) );
}