#ifndef SENSOR_FUSION_POINT_H
#define SENSOR_FUSION_POINT_H
#include <vector>

class SensorFusionPoint {

  public:

  	float d;
    double vx;
    double vy;
    double speed;
    double s;

    std::vector<SensorFusionPoint> generate_predictions(void){
    	std::vector<SensorFusionPoint> predictions;

    	// TODO add at least the position plus the speed*sampling time.
    	predictions.push_back(*this);

    	return predictions;
    };
};

#endif 
