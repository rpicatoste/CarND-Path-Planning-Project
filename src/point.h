
#ifndef POINT_H
#define POINT_H

#include <vector>
#include <string>

class Point{

public:
	double x;
	double y;

	double s;
	double d;
	double yaw_deg;
	double yaw_rad;

	double speed;


	int lane;
	double ref_vel;

	// Point(const Point &point);

	Point();
	Point(std::vector<double> xy);
	Point(double x, double y);
	Point(double x, double y, double s, double d);
	Point(double x, double y, double s, double d, double yaw, double speed);

	virtual ~Point();

	void print(std::string text = "");

	double distance(Point p1);
	double distance(Point p1, Point p2);

	Point& operator=(const Point& other_point){
		if (this != &other_point) { // self-assignment check expected
	        this->x = other_point.x;
			this->y = other_point.y;
			this->s = other_point.s;
			this->d = other_point.d;
			this->yaw_deg = other_point.yaw_deg;
			this->yaw_rad = other_point.yaw_rad;
			this->speed = other_point.speed;

	    }
	    return *this;
	}

	
  Point operator+(const Point& p1){
    Point sum = Point();
	sum.x = p1.x + this->x;
	sum.y = p1.y + this->y;

    return sum; 
  }

	
  Point operator-(const Point& p1){
    Point sum = Point();
	sum.x = this->x - p1.x;
	sum.y = this->y - p1.y;

    return sum; 
  }

  Point rotate_by_angle(double angle);

  double module(void);


  static std::vector<double> get_vector_x_from_list(std::vector<Point> &list){
  	std::vector<double> new_list;
  	for(std::vector<Point>::iterator p_point = list.begin(); p_point != list.end(); p_point++){
  		new_list.push_back(p_point->x);
  	}
  	return new_list;
  }

  static std::vector<double> get_vector_y_from_list(std::vector<Point> &list){
  	std::vector<double> new_list;
  	for(std::vector<Point>::iterator p_point = list.begin(); p_point != list.end(); p_point++){
  		new_list.push_back(p_point->y);
  	}
  	return new_list;
  }


};


#endif 
