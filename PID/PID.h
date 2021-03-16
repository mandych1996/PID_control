#ifndef PID_H_
#define PID_H_

#include <vector>
#include "Robot.h"

using namespace std;

class PID {
public:
	PID();
	virtual ~PID();

	ROBOT make_robot();
	vector<vector<double>> Run(ROBOT robot, vector<double> params, 
											int n=100, double speed =1.0);
	vector<double> twiddle(double tol =0.2);
};

#endif 