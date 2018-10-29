#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
using namespace Eigen;

Eigen::MatrixXd Y(Matrix3d R, Vector6d qd, Vector6d qd_d, Vector6d qdd_d);
Eigen::MatrixXd J(Matrix3d R, Matrix3d Rd);
Eigen::MatrixXd Jdot(Matrix3d R, Matrix3d Rd, Matrix3d R_dot, Matrix3d Rd_dot);
Vector3d p_des(double t);
Matrix3d R_des(double t);
Vector6d qd_d(double t);
Vector6d qdd_d(double t);
Eigen::Matrix3d cross(Eigen::Vector3d w);
Eigen::MatrixXd Minv(Eigen::Matrix3d R, Eigen::VectorXd a);
Eigen::MatrixXd Z(Eigen::Matrix3d R, Vector6d F);