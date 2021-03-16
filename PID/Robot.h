#ifndef ROBOT_H_
#define ROBOT_H_

#include <vector>
#define M_PI 3.1415926

using namespace std;

class ROBOT {
public:
	ROBOT();
	virtual ~ROBOT();
	void Init();
	void Set(double x, double y, double orientation);
	void SetNoise(double steering_noise, double distance_noise);
	void SetSteeringDrift(double steering_drift);
	void Move(double steering, double distance, double tolerance=0.001,
		double max_steering_angle = M_PI/4);


//private:
	double x;
	double y;
	double orientation;
	double length;
	double steering_noise;
	double distance_noise;
	double steering_drift;
};

#endif 