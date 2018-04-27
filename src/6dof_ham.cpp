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
#include "qDes.h"

using namespace Eigen;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 6, 1> Vector6d;

class Controller{
	int robotNum;
	double time;
	VectorXd a;
	Vector6d q, qd;
	Matrix6d Kd, L, Gamma;
	ros::Publisher cmd_pub;
	ros::Publisher state_pub;
	public:
		Controller(int, ros::NodeHandle);
		void stateCallback(const gazebo_msgs::LinkStates::ConstPtr& _msg);

};

Controller::Controller(int input, ros::NodeHandle n){
	srand(input);
	this->robotNum = input;
	this->time = 0;
	this->q = Vector6d::Zero();
	this->qd = Vector6d::Zero();
	this->a = VectorXd::Random(40);
	this->Gamma = 10*Matrix6d::Identity();
	this->Kd = 0.1*Matrix6d::Identity();
	this->L = 1*Matrix6d::Identity();
	this->cmd_pub = n.advertise<geometry_msgs::Wrench>("wrenchPlugin/wrenchCommands",1000); //remapped in launch file to correct topic
	this->state_pub = n.advertise<orbit_sim::state>("controllerState",1000);
};

void Controller::stateCallback(const gazebo_msgs::LinkStates::ConstPtr& _msg){
	if (this->time==0){
		//do nothing, gazebo not running yet
		this->time = ros::Time::now().toSec();
	} else {
		double currTime = ros::Time::now().toSec();
		//extract state
		geometry_msgs::Pose pose = _msg->pose[1]; //use measurements from first robot (body 1 in Gazebo)
		geometry_msgs::Twist twist = _msg->twist[1];
		//form quaternion for orientation
		Quaterniond sensorOrientation = Quaterniond(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
		//express angular velocity in body frame
		Vector3d angVel = sensorOrientation.normalized().toRotationMatrix().transpose()*Vector3d(twist.angular.x,twist.angular.y,twist.angular.z);
		Vector3d linVel = sensorOrientation.normalized().toRotationMatrix().transpose()*Vector3d(twist.linear.x,twist.linear.y,twist.linear.z);
		//extract position + gibbs vector
		Vector3d pos;
		pos << pose.position.x, pose.position.y, pose.position.z;
		Vector3d gibbs;
		gibbs << pose.orientation.x/pose.orientation.w, pose.orientation.y/pose.orientation.w, pose.orientation.z/pose.orientation.w;
		//pack q, qd
		Vector6d qCurr;
		qCurr << pos, gibbs;
		Vector6d qdCurr;
		qdCurr << linVel, angVel;
		//get reference trajectory
		Vector6d q_des, qd_des, qdd_des;
		//NEED TO PUT THESE IN BODY FRAME!
		//Also think about the rotation in gibbs vector components (not euler angles)
		q_des = q_d(currTime);
		qd_des = qd_d(currTime);
		qdd_des = qdd_d(currTime);
		//filter position/velocity
		this->q = .2*qCurr + .8*this->q;
		this->qd = 0.2*qdCurr + 0.8*this->qd;
		//update adaptive controller params
		Vector6d q_t = this->q - q_des; //tracking error
		Vector6d qd_t = this->qd - qd_des;
		Vector6d s = qd_t + this->L*q_t;
		double dt = currTime - this->time;

		Vector6d sDead; //deadband error signal
		for(int i = 0; i < 6; i++){
			if(std::abs(s(i))<0.1){
				sDead(i) = 0.0;
			} else {
				sDead(i) = s(i);
			}
		}

		MatrixXd Y = Ybody(this->q,this->qd,qd_des,qdd_des);

		Vector6d u = Y*this->a - this->Kd*s; //control vector in body frame
		this->a = this->a - dt*(this->Gamma*Y.transpose()*s);

		//need to put force into body frame
		Quaterniond bodyOrientation = Quaterniond(_msg->pose[0].orientation.w,_msg->pose[0].orientation.x,_msg->pose[0].orientation.y,_msg->pose[0].orientation.z);
		Matrix3d sensorToBody = bodyOrientation.normalized().toRotationMatrix().transpose()*sensorOrientation.normalized.toRotationMatrix();

		Vector3d fBody = sensorToBody*Vector3d(u(0),u(1),u(2));
		Vector3d tBody = sensorToBody*Vector3d(u(3),u(4),u(5));

		geometry_msgs::Wrench cmdWrench;
		geometry_msgs::Vector3 force;
		geometry_msgs::Vector3 torque;
		orbit_sim::state state_msg;

		force.x = fBody(0);
		force.y = fBody(1);
		force.z = fBody(2);

		torque.x = tBody(0);
		torque.y = tBody(1);
		torque.z = tBody(2);

		cmdWrench.force = force;
		cmdWrench.torque = torque;

		std::vector<double> xmsg;
		std::vector<double> xm_msg;
		for(int i = 0; i < 6; i++){
			xmsg.push_back(this->q(i));
			xm_msg.push_back(this->q_d(i));
		}
		state_msg.x = xmsg;
		state_msg.xm = xm_msg;

		this->cmd_pub.publish(cmdWrench); //this should be in body frame
		this->state_pub.publish(state_msg);
		this->time = currTime;
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