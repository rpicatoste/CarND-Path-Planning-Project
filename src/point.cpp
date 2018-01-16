#include <iostream>

#include <math.h>
#include "point.h"
#include "helpers.h"

Point::Point()
{
	this->ref_vel = 0.0;
}

Point::Point(std::vector<double> xy)
{
	this->x = xy[0];
	this->y = xy[1];

	this->ref_vel = 0.0;
}

Point::Point(double x, double y)
{
	this->x = x;
	this->y = y;

	this->ref_vel = 0.0;
}

Point::Point(double x, double y, double s, double d)
{
	this->x  = x;
	this->y  = y;
	this->s  = s;
	this->d  = d;

	this->ref_vel = 0.0;
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

	this->ref_vel = 0.0;
}

// Point::Point(const Point &point)
// {
// 	*this = point;

// }

Point::~Point() 
{

}


void Point::print(std::string text)
{
	std::cout << text;
	std::cout << "Point (x: " << this->x << ", y: " << this->y << ", s: " << this->s << ", d: " << this->d  << ", yaw_deg: " << this->yaw_deg << ")." << std::endl;

}


double Point::module(void)
{
	Point zero = Point(0.0, 0.0);

	return this->distance(zero);
}

double Point::distance(Point p)
{
	return distance(*this, p);
}


double Point::distance(Point p1, Point p2)
{
	return sqrt( (p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y) );
}

Point Point::rotate_by_angle(double angle)
{
	Point rotated = Point(*this);

	rotated.x = (this->x * cos(angle) - this->y * sin(angle));
	rotated.y = (this->x * sin(angle) + this->y * cos(angle));

	return rotated;
}