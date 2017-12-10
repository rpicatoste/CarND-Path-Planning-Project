
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"

// For converting back and forth between radians and degrees.


constexpr double pi() { return M_PI; }

double deg2rad(double x)
{ 
	return x * pi() / 180.0; 
}
double rad2deg(double x) 
{ 
	return x * 180.0 / pi(); 
}