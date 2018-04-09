#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "orbit_sim/refSig6.h"
#include <stdlib.h>
#include <sstream>
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "orbit_sim/state.h"
#include "Ybody.h"

using namespace Eigen;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 6, 1> Vector6d;

class Controller{
	int robotNum;
	double stateTime;
	double refTime;
	Vector6d x, xM, r,xMFilt, rFilt;
	Matrix6d Kx, Kr, gX, gR, gJ, gL, Am, Bm, P, sX, sR, sJ, sL;
	MatrixXd J;
	MatrixXd L;
	ros::Publisher cmd_pub;
	ros::Publisher state_pub;
	public:
		Controller(int, ros::NodeHandle);
		void refCallback(const orbit_sim::refSig6::ConstPtr& _msg);
		void stateCallback(const gazebo_msgs::LinkStates::ConstPtr& _msg);
		Vector6d f(Vector6d x);
		Vector6d g(Vector6d x);

};

Controller::Controller(int input, ros::NodeHandle n){
	srand(input);
	this->robotNum = input;
	this->stateTime = 0;
	this->refTime = 0;
	this->x << 0,0,0,0,0,0;
	this->xM << 0,0,0,0,0,0;
	this->xMFilt << 0,0,0,0,0,0;
	this->rFilt << 0,0,0,0,0,0;
	this->r << 0,0,0,0,0,0;
	this->Kx = 1e-15*MatrixXd::Identity(6,6);
	this->Kr = 1e-15*MatrixXd::Identity(6,6);
	this->J = 1e-15*MatrixXd::Ones(6,6);
	this->L = 1e-15*MatrixXd::Ones(6,6);
	this->gX = 1e3*MatrixXd::Identity(6,6);
	this->gR = 1e3*MatrixXd::Identity(6,6);
	this->gJ = 5e4*MatrixXd::Identity(6,6);
	this->gL = 5e4*MatrixXd::Identity(6,6);
	this->sX = 1e0*MatrixXd::Identity(6,6);
	this->sR = 1e0*MatrixXd::Identity(6,6);
	this->sJ = 5e0*MatrixXd::Identity(6,6);
	this->sL = 5e0*MatrixXd::Identity(6,6);
	this->Am = -0.01*MatrixXd::Identity(6,6);
	this->Bm = MatrixXd::Identity(6,6);
	this->P = 5*MatrixXd::Identity(6,6);
	this->cmd_pub = n.advertise<geometry_msgs::Wrench>("wrenchPlugin/wrenchCommands",1000);
	this->state_pub = n.advertise<orbit_sim::state>("controllerState",1000);
};

Vector6d Controller::f(Vector6d x){
	Vector6d fO;
	fO << pow(x(3),2),pow(x(4),2),pow(x(5),2),x(3)*x(4),x(3)*x(5),x(4)*x(5);
	return fO;
}

Vector6d Controller::g(Vector6d x){
	Vector6d gO;
	gO << x(3)*x(1), x(3)*x(2), x(4)*x(0), x(4)*x(2), x(5)*x(0), x(5)*x(1);
	return gO;
}

void Controller::refCallback(const orbit_sim::refSig6::ConstPtr& _msg){
	if (refTime == 0){
		this->xM << Vector6d::Map(_msg->xm.data(),6);
		this->r << Vector6d::Map(_msg->r.data(),6);
		this->refTime = ros::Time::now().toSec();
	} else {
		this->xM << Vector6d::Map(_msg->xm.data(),6);
		this->r << Vector6d::Map(_msg->r.data(),6);
		this->xMFilt = .5*this->xM+ .5*this->xMFilt;
		this->rFilt = .5*this->r + .5*this->rFilt;
		this->refTime = ros::Time::now().toSec();
	}
	
}

void Controller::stateCallback(const gazebo_msgs::LinkStates::ConstPtr& _msg){
	if (this->stateTime==0){
		//do nothing, gazebo not running yet
		this->stateTime = ros::Time::now().toSec();
	} else {
		double currTime = ros::Time::now().toSec();
		//extract state
		geometry_msgs::Pose pose = _msg->pose[0];
		geometry_msgs::Twist twist = _msg->twist[0];
		//form quaternion for orientation
		Quaterniond orientation = Quaterniond(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
		//express angular velocity in robot frame
		Vector3d angVel = orientation.normalized().toRotationMatrix().transpose()*Vector3d(twist.angular.x,twist.angular.y,twist.angular.z);
		Vector3d linVel = orientation.normalized().toRotationMatrix().transpose()*Vector3d(twist.linear.x,twist.linear.y,twist.linear.z);
		
		Vector6d xCurr;
		xCurr << linVel, angVel;
		this->x = .2*xCurr + .8*this->x;
		//update adaptive controller params
		Vector6d e = this->x - this->xMFilt; //tracking error
		double dt = currTime - this->stateTime;

		Vector6d eDead;
		for(int i = 0; i < 6; i++){
			if(std::abs(e(i))<0.001){
				eDead(i) = 0.0;
			} else {
				eDead(i) = e(i);
			}
		}

		Vector6d u = this->Kx*this->x + this->Kr*this->rFilt + this->J*(this->f(this->x))+ this->L*(this->g(this->x)); //control vector in body frame


		this->Kx = this->Kx + dt*(-this->gX*this->Bm.transpose()*this->P*eDead*this->x.transpose()-this->sX*this->Kx*e.norm());
		this->Kr = this->Kr + dt*(-this->gR*this->Bm.transpose()*this->P*eDead*this->rFilt.transpose()-this->sR*this->Kr*e.norm());
		this->J = this->J + dt*(-this->gJ*this->Bm.transpose()*this->P*eDead*(this->f(this->x)).transpose()-this->sJ*this->J*e.norm());
		this->L = this->L + dt*(-this->gL*this->Bm.transpose()*this->P*eDead*(this->g(this->x)).transpose()-this->sL*this->L*e.norm());


		//Quaterniond bodyOrientation = Quaterniond(_msg->pose[0].orientation.x,_msg->pose[0].orientation.y,_msg->pose[0].orientation.z,_msg->pose[0].orientation.w);

		//Vector3d uBody = bodyOrientation.toRotationMatrix()*orientation.toRotationMatrix().transpose()*u;

		geometry_msgs::Wrench cmdWrench;
		geometry_msgs::Vector3 force;
		geometry_msgs::Vector3 torque;
		orbit_sim::state state_msg;

		force.x = u(0);
		force.y = u(1);
		force.z = u(2);

		torque.x = u(3);
		torque.y = u(4);
		torque.z = u(5);

		cmdWrench.force = force;
		cmdWrench.torque = torque;

		std::vector<double> xmsg;
		std::vector<double> xm_msg;
		for(int i = 0; i < 6; i++){
			xmsg.push_back(this->x(i));
			xm_msg.push_back(this->xMFilt(i));
		}
		state_msg.x = xmsg;
		state_msg.xm = xm_msg;

		this->cmd_pub.publish(cmdWrench);
		this->state_pub.publish(state_msg);
		this->stateTime = currTime;
	}
}


int main(int argc, char *argv[]){
	std::stringstream ss;
	ss << "controller" << argv[1];
	ros::init(argc,argv,ss.str());
	ros::NodeHandle n;
	Controller ctrl (atoi(argv[1]),n);
	ROS_INFO("all good!");
	ros::Subscriber refSub = n.subscribe("refSignal", 1, &Controller::refCallback, &ctrl);
	ros::Subscriber stateSub = n.subscribe("/gazebo/link_states",1, &Controller::stateCallback, &ctrl);
	ros::spin();
	return 0;
}