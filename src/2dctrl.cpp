#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
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
#include "std_msgs/String.h"

using namespace Eigen;

class Controller{
	int robotNum;
	double stateTime;
	Vector3d x, xM, r;
	Matrix3d Kx, Kr, gamX, gamR, gamM, gamL, Am, Bm, P, wX, wR, wL, wM;
	MatrixXd L, M;
	ros::Publisher cmd_pub;
	ros::Publisher state_pub;
	ros::Publisher model_pub;
	public:
		Controller(int, ros::NodeHandle);
		void refCallback(const orbit_sim::refSig::ConstPtr& _msg);
		void stateCallback(const gazebo_msgs::LinkStates::ConstPtr& _msg);
		VectorXd fX(Vector3d x);
		VectorXd gX(Vector3d x);
};

Controller::Controller(int input, ros::NodeHandle n){
	srand(input);
	this->robotNum = input;
	this->stateTime = 0;
	this->x = Vector3d(0,0,0);
	this->xM = Vector3d(0,0,0);
	this->r = Vector3d(0,0,0);
	this->Kx = 1e1*MatrixXd::Identity(3,3);
	this->Kr = 1e1*MatrixXd::Identity(3,3);
	this->L = 1e-10*MatrixXd::Identity(3,3);
	this->M = 1e-10*MatrixXd::Ones(3,2);
	this->gamX = 2e1*MatrixXd::Identity(3,3);
	this->gamR = 2e1*MatrixXd::Identity(3,3);
	this->gamM = 1e-2*MatrixXd::Identity(3,3);
	this->gamL = 1e-2*MatrixXd::Identity(3,3);
	this->wX = 1e-20*MatrixXd::Identity(3,3);
	this->wR = 1e-20*MatrixXd::Identity(3,3);
	this->wL = 1e-20*MatrixXd::Identity(3,3);
	this->wM = 1e-20*MatrixXd::Identity(3,3);
	this->Am = -0.01*MatrixXd::Identity(3,3);
	this->Bm = MatrixXd::Identity(3,3);
	this->P = 5*MatrixXd::Identity(3,3);
	this->cmd_pub = n.advertise<geometry_msgs::Wrench>("wrenchPlugin/wrenchCommands",1000);
	this->state_pub = n.advertise<geometry_msgs::Vector3>("controllerState",1000);
	this->model_pub = n.advertise<geometry_msgs::Vector3>("modelState",1000);
};

VectorXd Controller::fX(Vector3d x){
	VectorXd outVec(3);
	outVec << copysign(1.0,x(0)), copysign(1.0,x(1)), copysign(1.0,x(2));
	return outVec;
}

VectorXd Controller::gX(Vector3d x){
	VectorXd outVec(2);
	outVec << x(0)*x(2), x(1)*x(2);
	return outVec;
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
		geometry_msgs::Pose pose = _msg->pose[1];
		geometry_msgs::Twist twist = _msg->twist[1];
		//form quaternion for orientation
		Quaterniond orientation = Quaterniond(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
		//express angular velocity in robot frame
		Vector3d angVel = orientation.normalized().toRotationMatrix().transpose()*Vector3d(twist.angular.x,twist.angular.y,twist.angular.z);
		Vector3d linVel = orientation.normalized().toRotationMatrix().transpose()*Vector3d(twist.linear.x,twist.linear.y,twist.linear.z);

		this->x = Vector3d(linVel(0),linVel(1),angVel(2));


		//update adaptive controller params
		Vector3d e = this->x - this->xM; //tracking error
		double dt = currTime - this->stateTime;

		this->Kx = this->Kx + dt*(-this->gamX*this->Bm.transpose()*this->P*e*this->x.transpose()-this->wX*this->Kx);
		this->Kr = this->Kr + dt*(-this->gamR*this->Bm.transpose()*this->P*e*this->r.transpose()-this->wR*this->Kr);
		this->L = this->L + dt*(-this->gamL*this->Bm.transpose()*this->P*e*(this->fX(this->x)).transpose()-this->wL*this->L);
		this->M = this->M + dt*(-this->gamM*this->Bm.transpose()*this->P*e*(this->gX(this->x)).transpose()-this->wM*this->M);

		Vector3d u = this->Kx*this->x + this->Kr*this->r + this->L*(this->fX(this->x))+ this->M*(this->gX(this->x)); //control vector in robot frame

		//Quaterniond bodyOrientation = Quaterniond(_msg->pose[0].orientation.x,_msg->pose[0].orientation.y,_msg->pose[0].orientation.z,_msg->pose[0].orientation.w);

		//Vector3d uBody = bodyOrientation.toRotationMatrix()*orientation.toRotationMatrix().transpose()*u;

		geometry_msgs::Wrench cmdWrench;
		geometry_msgs::Vector3 force;
		geometry_msgs::Vector3 torque;
		geometry_msgs::Vector3 state;
		geometry_msgs::Vector3 model;

		force.x = u(0);
		force.y = u(1);
		force.z = 0;

		torque.x = 0;
		torque.y = 0;
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