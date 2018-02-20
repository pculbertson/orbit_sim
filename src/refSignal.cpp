#include <eigen3/Eigen/Dense>
#include <math.h>
#include <random>
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include <orbit_sim/refSig.h>

using namespace Eigen;

class RefSignal
{
	double refTime;
	Vector3d xM, r;
	Matrix3d Am;
	Matrix3d Bm;
	public:
		RefSignal();
		void update();
		Vector3d getX() {return xM;}
		Vector3d getR() {return r;}
};

RefSignal::RefSignal(){
	this->refTime = ros::Time::now().toSec();
	this->xM = Vector3d(0,0,0);
	this->r = Vector3d(0,0,0);
	this->Am = -0.01*MatrixXd::Identity(3,3);
	this->Bm = MatrixXd::Identity(3,3);
}

void RefSignal::update(){
	std::default_random_engine generator;
  	std::normal_distribution<double> distribution(0.0,.1);
	double currTime = ros::Time::now().toSec();
	//this->r = (distribution(generator)+.25*sin(currTime)+.25*sin(2*currTime)+.25*sin(3*currTime)+.25*sin(10*currTime)+.1*sin(50*currTime))*Vector3d(1,1,1);
	this->r = (.25*sin(currTime)+.25*sin(2*currTime)+.25*sin(3*currTime)+.25*sin(10*currTime))*Vector3d(1,1,1);
	this->xM = this->xM + (currTime-refTime)*(Am*this->xM + Bm*this->r);
	this->refTime = currTime;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "refSig");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<orbit_sim::refSig>("refSignal", 1000);
	ros::Rate loop_rate(500);

	RefSignal rS;

	while (ros::ok())
	{
		/**
		* This is a message object. You stuff it with data, and then publish it.
		*/
		orbit_sim::refSig msg;

		rS.update();

		Vector3d currX = rS.getX();
		Vector3d currR = rS.getR();

		msg.xm.x = currX(0);
		msg.xm.y = currX(1);
		msg.xm.z = currX(2);

		msg.r.x = currR(0);
		msg.r.y = currR(1);
		msg.r.z = currR(2);
	    
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