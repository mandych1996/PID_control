#include <iostream>
#include <numeric>
#include "PID.h"

using namespace std;

PID::PID() {}
PID::~PID() {}

ROBOT PID::make_robot() {
	ROBOT robot = ROBOT();
	robot.Init();
	robot.Set(0, 1, 0);
	robot.SetSteeringDrift(10/180 *M_PI);
	return robot;
}

vector<vector<double>> PID::Run(ROBOT robot, vector<double> params,
	int n, double speed) {

	vector<double> x_trajectory;
	vector<double> y_trajectory;
	vector<double> error(1, 0.0);
	vector<vector<double>> output;
	double prev_cte = robot.y;
	double cte_i = 0.0;

	for (int i = 0; i < 2 * n; i++) {
		double cte = robot.y;
		double diff_cte = cte - prev_cte;
		cte_i += cte;
		prev_cte = cte;
		double steer = -params[0] * cte - params[1] * diff_cte - params[2] * cte_i;
		robot.Move(steer, speed);
		x_trajectory.push_back(robot.x);
		y_trajectory.push_back(robot.y);

		if (i >= n) error[0] += cte * cte;
	}
	output.push_back(x_trajectory);
	output.push_back(y_trajectory);
	output.push_back(error);

	return output;
}

vector<double> PID::twiddle(double tol) {
	vector<double> p = {0, 0, 0};
	vector<double> dp = {1.0, 1.0, 1.0};
	ROBOT robot = make_robot();
	vector<vector<double>> run_output = Run(robot, p);
	vector<double> x_trajectory = run_output[0];
	vector<double> y_trajectory = run_output[1];
	double best_err = run_output[2][0];
	double err;
	int iteration = 0;

	while (accumulate(dp.begin(), dp.end(), 0.0) > tol) {
		cout << "Iteration " << iteration << ", best error = " << best_err << endl;
		for (int i = 0; i < p.size(); i++) {
			p[i] += dp[i];
			robot = make_robot();
			run_output = Run(robot, p);
			x_trajectory = run_output[0];
			y_trajectory = run_output[1];
			err = run_output[2][0];

			if (err < best_err) {
				best_err = err;
				dp[i] *= 1.1;
			}
			else {
				p[i] -= 2 * dp[i];
				robot = make_robot();
				run_output = Run(robot, p);
				x_trajectory = run_output[0];
				y_trajectory = run_output[1];
				err = run_output[2][0];
				
				if (err < best_err) {
					best_err = err;
					dp[i] = dp[i] *1.1;
				}
				else {
					p[i] += dp[i];
					dp[i] = dp[i] *0.9;
				}
			}
		}
		cout << accumulate(dp.begin(), dp.end(), 0.0) << endl;
		cout << tol << endl;
		iteration += 1;
		
	}
	p.push_back(best_err);
	return p;
}
