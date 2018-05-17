#include <eigen3/Eigen/Dense>

extern double f;
extern double r;

typedef Eigen::Matrix<double, 6, 1> Vector6d;

using namespace Eigen;

Vector6d breaks((Vector6d() << 0, 4, 8, 12, 16, 20).finished());

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

MatrixXd rotCoefs((MatrixXd(15,4) << 	0.0028,   	-0.0774,	0.4421,    0,
							   			-0.0064,    0.0644,    	0.0223,    0,
							        	0,         	0,         	0,         0,
							    		0.0028,   	-0.0442,   	-0.0443,   0.7071,
							   			-0.0064,  	-0.0129,    0.2281,    0.7071,
							         	0,         	0,         	0,         0,
							    		0.0041,   	-0.0110,   	-0.2650,   0,
							    		0.0147,   	-0.0903,   	-0.1848,   1.0000,
							         	0,         	0,         	0,         0,
							   			-0.0005,    0.0388,   	-0.1538,   -0.9701,
							   			-0.0052,    0.0863,   	-0.2010,   -0.2425,
							         	0,         	0,        	0,         0,
							   			-0.0005,    0.0322,    	0.1300,    -1.0000,
							   			-0.0052,    0.0237,    	0.2388,    0,
							         	0,         	0,         	0,         0).finished());

Vector6d q_d(double t,Eigen::Quaterniond q);
Vector6d qd_d(double t,Eigen::Quaterniond q);
Vector6d qdd_d(double t,Eigen::Quaterniond q);