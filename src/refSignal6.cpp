#include <Eigen/Dense>
#include <math.h>
#include <random>
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include <orbit_sim/refSig6.h>

using namespace Eigen;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 6, 1> Vector6d;

class RefSignal
{
	double refTime;
	Vector6d xM, r;
	Matrix6d Am;
	Matrix6d Bm;
	public:
		RefSignal();
		void update();
		Vector6d getX() {return xM;}
		Vector6d getR() {return r;}
};

RefSignal::RefSignal(){
	this->refTime = ros::Time::now().toSec();
	this->xM << 0.0,0.0,0.0,0.0,0.0,0.0;
	this->r << 0.0,0.0,0.0,0.0,0.0,0.0;
	this->Am = -0.01*MatrixXd::Identity(6,6);
	this->Bm = MatrixXd::Identity(6,6);
}

void RefSignal::update(){
	double currTime = ros::Time::now().toSec();
	//this->r = (distribution(generator)+.25*sin(currTime)+.25*sin(2*currTime)+.25*sin(3*currTime)+.25*sin(10*currTime)+.1*sin(50*currTime))*Vector3d(1,1,1);
	Vector6d sigVec;
	sigVec << 1.0,1.0,0.0,1.0,0.0,0.0;
	this->r = .05*(.25*sin(currTime)+.25*sin(2*currTime)+.25*sin(3*currTime)+.25*sin(10*currTime))*sigVec;
	this->xM = this->xM + (currTime-refTime)*(Am*this->xM + Bm*this->r);
	this->refTime = currTime;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "refSig");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<orbit_sim::refSig6>("refSignal", 1000);
	ros::Rate loop_rate(500);

	RefSignal rS;

	while (ros::ok())
	{
		/**
		* This is a message object. You stuff it with data, and then publish it.
		*/
		orbit_sim::refSig6 msg;

		rS.update();

		Vector6d currX = rS.getX();
		Vector6d currR = rS.getR();

		std::vector<double> xmsg;
		std::vector<double> rmsg;

		for(int i=0; i < 6; i++){
			xmsg.push_back(currX(i));
			rmsg.push_back(currR(i));
		}

		msg.xm = xmsg;
		msg.r = rmsg;
	    
	    /**
	     * The publish() function is how you send messages. The parameter
	     * is the message object. The type of this object must agree with the type
	     * given as a template parameter to the advertise<>() call, as was done
	     * in the constructor above.
	     */
	    chatter_pub.publish(msg);

	    ros::spinOnce();

	    loop_rate.sleep();

	}
	return 0;
}