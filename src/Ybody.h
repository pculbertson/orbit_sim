#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

typedef Matrix<double, 6, 1> Vector6d;

MatrixXd Ybody(Vector6d q, Vector6d dq, Vector6d dq_d, Vector6d ddq_d);