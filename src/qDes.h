#include <eigen3/Eigen/Dense>

extern double f;
extern double r;

typedef Eigen::Matrix<double, 6, 1> Vector6d;

Vector6d q_d(double t);
Vector6d qd_d(double t);
Vector6d qdd_d(double t);