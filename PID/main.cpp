#include "PID.h"
#include "Robot.h"

int main() {


	
	PID pid = PID();
	vector<double> p_and_error =  pid.twiddle();
	double Kp = p_and_error[0];
	double Kd = p_and_error[1];
	double Ki = p_and_error[2];
	double err = p_and_error[3];
	vector<double> p = {Kp, Kd, Ki};

	ROBOT robot = pid.make_robot();
	vector<vector<double>> run_output = pid.Run(robot, p);
	vector<double> x_trajectory = run_output[0];
	vector<double> y_trajectory = run_output[1];
	double best_err = run_output[2][0];
	

	system("pause");
	return 0;
}