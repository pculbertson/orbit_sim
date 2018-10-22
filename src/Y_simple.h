#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

Eigen::MatrixXd Y(Matrix3d R, Vector6d qd, Vector6d qd_d, Vector6d qdd_d);