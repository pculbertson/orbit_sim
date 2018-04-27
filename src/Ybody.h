#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

Eigen::MatrixXd Ybody(Vector6d q, Vector6d dq, Vector6d dq_d, Vector6d ddq_d);