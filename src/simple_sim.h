#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

Eigen::MatrixXd Y(Eigen::Matrix3d R, Vector6d qd, Vector6d qd_d, Vector6d qdd_d);
Vector6d qd_d(double t);
Vector6d qdd_d(double t);
Eigen::Matrix3d cross(Eigen::Vector3d w);
Eigen::MatrixXd Minv(Eigen::Matrix3d R, Eigen::VectorXd a);
Eigen::MatrixXd Z(Eigen::Matrix3d R, Vector6d F);