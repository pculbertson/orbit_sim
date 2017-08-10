#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <math.h>
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "orbit_sim/refSig.h"
#include <stdlib.h>
#include <sstream>
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

using namespace Eigen;

class Controller{
	int robotNum;
	double stateTime;
	Vector3d x, xM, r;
	Matrix3d Kx, Kr, gX, gR, gJ, Am, Bm, P;
	MatrixXd J;
	ros::Publisher cmd_pub;
	ros::Publisher state_pub;
	ros::Publisher model_pub;
	public:
		Controller(int, ros::NodeHandle);
		void refCallback(const orbit_sim::refSig::ConstPtr& _msg);
		void stateCallback(const gazebo_msgs::LinkStates::ConstPtr& _msg);
		VectorXd fOmega(Vector3d x);
};

Controller::Controller(int input, ros::NodeHandle n){
	srand(input);
	this->robotNum = input;
	this->stateTime = 0;
	this->x = Vector3d(0,0,0);
	this->xM = Vector3d(0,0,0);
	this->r = Vector3d(0,0,0);
	this->Kx = .01*MatrixXd::Identity(3,3);
	this->Kr = .01*MatrixXd::Identity(3,3);
	this->J = 1e-4*MatrixXd::Ones(3,6);
	this->gX = 1e4*MatrixXd::Identity(3,3);
	this->gR = 1e4*MatrixXd::Identity(3,3);
	this->gJ = 1e0*MatrixXd::Identity(3,3);
	this->Am = -0.01*MatrixXd::Identity(3,3);
	this->Bm = MatrixXd::Identity(3,3);
	this->P = 5*MatrixXd::Identity(3,3);
	this->cmd_pub = n.advertise<geometry_msgs::Wrench>("wrenchPlugin/wrenchCommands",1000);
	this->state_pub = n.advertise<geometry_msgs::Vector3>("controllerState",1000);
	this->model_pub = n.advertise<geometry_msgs::Vector3>("modelState",1000);
};

VectorXd Controller::fOmega(Vector3d x){
	VectorXd fO(6);
	fO << pow(x(0),2),pow(x(1),2),pow(x(2),2),x(0)*x(1),x(0)*x(2),x(1)*x(2);
	return fO;
}

void Controller::refCallback(const orbit_sim::refSig::ConstPtr& _msg){
	this->xM = Vector3d(_msg->xm.x,_msg->xm.y,_msg->xm.z);
	this->r = Vector3d(_msg->r.x,_msg->r.y,_msg->r.z);
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
		Quaterniond orientation = Quaterniond(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
		//express angular velocity in robot frame
		this->x = orientation.normalized().toRotationMatrix().transpose()*Vector3d(twist.angular.x,twist.angular.y,twist.angular.z);
		//update adaptive controller params
		Vector3d e = this->x - this->xM; //tracking error
		double dt = currTime - this->stateTime;

		this->Kx = this->Kx + dt*(-this->gX*this->Bm.transpose()*this->P*e*this->x.transpose()-5*this->Kx);
		this->Kr = this->Kr + dt*(-this->gR*this->Bm.transpose()*this->P*e*this->r.transpose()-5*this->Kr);
		this->J = this->J + dt*(-this->gJ*this->Bm.transpose()*this->P*e*(this->fOmega(this->x)).transpose()-.5*this->J);

		Vector3d u = this->Kx*this->x + this->Kr*this->r + this->J*(this->fOmega(this->x)); //control vector in robot frame

		Quaterniond bodyOrientation = Quaterniond(_msg->pose[0].orientation.x,_msg->pose[0].orientation.y,_msg->pose[0].orientation.z,_msg->pose[0].orientation.w);

		//Vector3d uBody = bodyOrientation.toRotationMatrix()*orientation.toRotationMatrix().transpose()*u;

		geometry_msgs::Wrench cmdWrench;
		geometry_msgs::Vector3 force;
		geometry_msgs::Vector3 torque;
		geometry_msgs::Vector3 state;
		geometry_msgs::Vector3 model;

		force.x = 0;
		force.y = 0;
		force.z = 0;

		torque.x = u(0);
		torque.y = u(1);
		torque.z = u(2);

		cmdWrench.force = force;
		cmdWrench.torque = torque;

		state.x = this->x(0);
		state.y = this->x(1);
		state.z = this->x(2);

		model.x = this->xM(0);
		model.y = this->xM(1);
		model.z = this->xM(2);

		this->cmd_pub.publish(cmdWrench);
		this->state_pub.publish(state);
		this->model_pub.publish(model);
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