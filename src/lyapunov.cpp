#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <math.h>
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "orbit_sim/refSig.h"
#include <stdlib.h>
#include <vector>
#include <sstream>
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

using namespace Eigen;
class Lyapunov{
	double V;
	std::vector<MatrixXd> Kx;
	std::vector<MatrixXd> Kr;
	std::vector<MatrixXd> M;
	MatrixXd KxStar;
	MatrixXd KrStar;
	MatrixXd MStar;
}