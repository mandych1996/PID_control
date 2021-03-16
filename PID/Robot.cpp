#include <iostream>
#include <string>
#include <random>
#include <math.h>
#include <stdlib.h>

#include "Robot.h"

using namespace std;

ROBOT::ROBOT() {}
ROBOT::~ROBOT() {}

void ROBOT::Init() {
	x = 0.0;
	y = 0.0;
	orientation = 0.0;
	length = 20.0;
	steering_noise = 0.01;
	distance_noise = 0.01;
	steering_drift = 0.0;
}

void ROBOT::Set(double x, double y, double orientation) {
	this->x = x;
	this->y = y;
	this->orientation = orientation;
}

void ROBOT::SetNoise(double steering_noise, double distance_noise) {
	this->steering_noise = steering_noise;
	this->distance_noise = distance_noise;
}

void ROBOT::SetSteeringDrift(double steering_drift) {
	this->steering_drift = steering_drift;
}

void ROBOT::Move(double steering, double distance, double tolerance,
	double max_steering_angle) {

	if (steering > max_steering_angle) steering = max_steering_angle;
	if (steering < -max_steering_angle) steering = -max_steering_angle;
	if (distance < 0.0) distance = 0.0;

	default_random_engine generator;
	normal_distribution<double> distribution_steering(steering, steering_noise);
	normal_distribution<double> distribution_distance(distance, distance_noise);

	double steering2 = distribution_steering(generator);
	double distance2 = distribution_distance(generator);

	steering2 += this->steering_drift;
	double turn = tan(steering2)* distance2 / this->length;

	if (abs(turn) < tolerance) {
		this->x += distance2 * cos(this->orientation);
		this->y += distance2 * sin(this->orientation);
		this->orientation = this->orientation +turn;
		while (this->orientation > 2 * M_PI) this->orientation -= 2 * M_PI;
		while (this->orientation < 0) this->orientation += 2 * M_PI;
	}
	else {
		double radius = distance2 / turn;
		double cx = this->x - (sin(this->orientation) * radius);
		double cy = this->y + (cos(this->orientation) * radius);
		this->orientation = this->orientation + turn;
		while (this->orientation > 2 * M_PI) this->orientation -= 2 * M_PI;
		while (this->orientation < 0) this->orientation += 2 * M_PI;
		this->x = cx + (sin(this->orientation) * radius);
		this->y = cy - (cos(this->orientation) * radius);
	}
}