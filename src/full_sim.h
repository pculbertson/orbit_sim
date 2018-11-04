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

MatrixXd linCoefs((MatrixXd(15,4) <<   -0.0129,    0.1139,         0,         0,
									    0.0138,   -0.1175,         0,         0,
									   -0.0007,    0.0027,         0,         0,
									    0.0073,   -0.0404,    0.2943,    1.0000,
									   -0.0022,    0.0475,   -0.2799,   -1.0000,
									    0.0020,   -0.0053,   -0.0107,         0,
									   -0.0164,    0.0475,    0.3230,    2.0000,
									    0.0029,    0.0211,   -0.0054,   -1.5000,
									   -0.0022,    0.0187,    0.0428,         0,
									    0.0272,   -0.1498,   -0.0861,    3.0000,
									   -0.0015,    0.0556,    0.3014,   -1.0000,
									    0.0016,   -0.0076,    0.0871,    0.3300,
									   -0.0299,    0.1767,    0.0215,    2.0000,
									   -0.0203,    0.0377,    0.6746,    1.0000,
									   -0.0041,    0.0118,    0.1039,    0.6600).finished());

MatrixXd rotCoefs((MatrixXd(15,4) << 	0.0053,   -0.0211,         0,         0,
   										-0.0014,   0.0056,         0,         0,
									         0,         0,         0,         0,
									   -0.0036,    0.0423,    0.0846,         0,
									    0.0042,   -0.0113,   -0.0225,         0,
									         0,         0,         0,         0,
									   -0.0032,   -0.0007,    0.2508,    0.7854,
									   -0.0032,    0.0395,    0.0902,         0,
									         0,         0,         0,         0,
									    0.0042,   -0.0395,    0.0902,    1.5708,
									   -0.0036,    0.0007,    0.2508,    0.7854,
									         0,         0,         0,         0,
									   -0.0014,    0.0113,   -0.0225,    1.5708,
									    0.0053,   -0.0423,    0.0846,    1.5708,
									         0,         0,         0,         0).finished());