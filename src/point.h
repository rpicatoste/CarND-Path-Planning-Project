

class Point{

public:
	double x;
	double y;

	double s;
	double d;
	double yaw_deg;
	double yaw_rad;

	double speed;

	// Point(const Point &point);

	Point();
	Point(double x, double y);
	Point(double x, double y, double s, double d);
	Point(double x, double y, double s, double d, double yaw, double speed);

	virtual ~Point();

	void print(void);

	double distance(Point p1);
	double distance(Point p1, Point p2);

};