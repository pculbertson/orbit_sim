#include "qDes.h"
#include <eigen3/Eigen/Dense>
#include <math.h>

using namespace Eigen;
typedef Matrix<double, 6, 1> Vector6d;
double f = 0.5;
double r = 5;

Vector6d q_d(double t){
	Vector6d q_des;
	q_des << r*cos(f*t), r*sqrt(2.0)*sin(f*t)/2.0, r*sqrt(2.0)*sin(f*t)/2.0, sin(f*t), 0, 0;
	return q_des;
}

Vector6d qd_d(double t){
	Vector6d qd_des;
	qd_des << -r*sin(f*t), r*sqrt(2.0)*cos(f*t)/2.0, r*sqrt(2.0)*cos(f*t)/2.0, cos(f*t), 0, 0;
	return qd_des;
}

Vector6d qdd_d(double t){
	Vector6d qdd_des;
	qdd_des << -r*cos(f*t), -r*sqrt(2.0)*sin(f*t)/2.0, -r*sqrt(2.0)*sin(f*t)/2.0, -sin(f*t), 0, 0;
	return qd_des;
}