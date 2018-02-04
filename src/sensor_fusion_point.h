#ifndef SENSOR_FUSION_POINT_H
#define SENSOR_FUSION_POINT_H
#include <vector>

class SensorFusionPoint {

  public:

  	float d;
    double x;
    double y;
    double vx;
    double vy;
    double speed;
    double s;

    std::vector<SensorFusionPoint> generate_predictions(void){
    	std::vector<SensorFusionPoint> predictions;

    	predictions.push_back(*this);

    	return predictions;
    };
};

#endif 
