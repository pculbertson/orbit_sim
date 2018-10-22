#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "orbit_sim/refSig6.h"
#include <stdlib.h>
#include <sstream>
#include <string>
#include "std_msgs/Header.h"
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "orbit_sim/state.h"
#include "Ybody.h"
#include "qDes_const.h"

using namespace Eigen;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 6, 1> Vector6d;

class Controller{
	int robotNum, sensorNum;
	double time;
	double startTime;
	double db;
	VectorXd a;
	Vector6d q, qd;
	Matrix6d Kd, L;
	Matrix<double,40,40> Gamma;
	ros::Publisher cmd_pub;
	ros::Publisher state_pub;
	ros::Publisher path_pub;
	int seq;
	nav_msgs::Path path;
	geometry_msgs::PoseStamped pose;
	std_msgs::Header h;
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
	this->a = 1e-3*VectorXd::Random(18);
	sensorNum = -1;
	//this->Gamma = 0*Matrix<double,40,40>::Identity();
	double gW;
	n.getParam("/gW",gW);
	double kdW_l;
	n.getParam("/kdW_l",kdW_l);
	double kdW_a;
	n.getParam("/kdW_a",kdW_a);
	double lW;
	n.getParam("/lW",lW);
	n.getParam("/db",db);
	this->Gamma = gW*Matrix<double,18,18>::Identity();
	Vector6d KdVec;
	KdVec << kdW_l*MatrixXd::Ones(3,1), kdW_a*MatrixXd::Ones(3,1);
	this->Kd = KdVec.asDiagonal();
	this->L = lW*Matrix6d::Identity();
	this->cmd_pub = n.advertise<geometry_msgs::Wrench>("wrenchPlugin/wrenchCommands",1000); //remapped in launch file to correct topic
	this->state_pub = n.advertise<orbit_sim::state>("controllerState",1000);
	path_pub = n.advertise<nav_msgs::Path>("orbit_path",1000);
	h = std_msgs::Header();
	h.frame_id = "1";
	pose = geometry_msgs::PoseStamped();
	path = nav_msgs::Path();
	startTime = 0;
};

void Controller::stateCallback(const gazebo_msgs::LinkStates::ConstPtr& _msg){
	if (this->time==0){
		//do nothing, gazebo not running yet
		startTime = ros::Time::now().toSec();
		time = startTime;
	} else {
		double currTime = ros::Time::now().toSec()-startTime;
		double bodyNum;
		//extract state
		for (int i=0; i < _msg->name.size(); i++){
			if (_msg->name[i].compare("payload::body")==0){
				bodyNum = i;
			} else if (_msg->name[i].compare("payload::robot1")==0){
				sensorNum = i;
			}
		}
		geometry_msgs::Pose sPose = _msg->pose[sensorNum]; //use measurements from first robot (body 1 in Gazebo)
		geometry_msgs::Twist twist = _msg->twist[sensorNum];

		//form quaternion for orientation
		Quaterniond sensorOrientation = Quaterniond(sPose.orientation.w,sPose.orientation.x,sPose.orientation.y,sPose.orientation.z);
		sensorOrientation = sensorOrientation.normalized();
		Matrix3d sensorToWorld = sensorOrientation.normalized().toRotationMatrix().transpose();

		//get body orientation
		Quaterniond bodyOrientation = Quaterniond(_msg->pose[bodyNum].orientation.w,_msg->pose[bodyNum].orientation.x,_msg->pose[bodyNum].orientation.y,_msg->pose[bodyNum].orientation.z);
		Matrix3d worldToBody = bodyOrientation.normalized().toRotationMatrix();

		Vector3d angVel = Vector3d(twist.angular.x,twist.angular.y,twist.angular.z);
		Vector3d linVel = Vector3d(twist.linear.x,twist.linear.y,twist.linear.z); 
		
		Vector6d qdCurr;
		qdCurr << linVel, angVel; //linear & angular rates in world frame
		//get reference trajectory
		Vector6d qd_des, qdd_des;

		qd_des = qd_d(currTime);
		qdd_des = qdd_d(currTime);

		//filter position/velocity
		this->qd = 1*qdCurr + 0*this->qd;
		//update adaptive controller params
		Vector6d e = this->qd - qd_des; //tracking error
		double dt = currTime - this->time;

		Vector6d eDead; //deadband error signal
		for(int i = 0; i < 6; i++){
			if(std::abs(q_t(i))<db){
				eDead(i) = 0.0;
			} else {
				eDead(i) = s(i);
			}

		}

		MatrixXd Y = Y(sensorToWorld,this->qd,qd_r,qdd_r);

		Vector6d F = Y*this->a - this->Kd*eDead; //nominal control in world frame

		Vector6d tau = Minv(sensorToWorld,this->a)*F; //shift by estimate of M (in world frame)

		MatrixXd Z = Z(sensorToWorld,F);

		this->a = this->a - dt*(this->Gamma*(Y+Z).transpose()*eDead); //

		//need to put force into body frame
		Vector3d fBody = worldToBody*Vector3d(u(0),u(1),u(2));
		Vector3d tBody = worldToBody*Vector3d(u(3),u(4),u(5));
		//Vector3d fBody = Vector3d(u(0),u(1),u(2));
		//Vector3d tBody = Vector3d(u(3),u(4),u(5));

		//Vector3d fBody;
		//Vector3d tBody;

		//fBody << 0, 0, 0;
		//tBody << 0, 0, 0;

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
			xm_msg.push_back(q_des(i));
		}
		state_msg.x = xmsg;
		state_msg.xm = xm_msg;

		//express desired linear position in global frame
		//Vector3d q_des_lin = worldToBody.transpose()*q_des.head<3>();

		/*h.stamp = ros::Time::now();
		pose.header = h;
		path.header = h;
		pose.pose.position.x = q_des_lin(0);
		pose.pose.position.y = q_des_lin(1);
		pose.pose.position.z = q_des_lin(2);
		path.poses.push_back(pose);

		path_pub.publish(path);*/

		this->cmd_pub.publish(cmdWrench); //this should be in body frame
		this->state_pub.publish(state_msg);
		this->time = currTime;
	}
}

Matrix3d cross(Vector3d w){
	Matrix3d x;
	x << 0, -w(2), w(1),
		 w(2), 0, -w(0),
		 -w(1), w(0), 0;
	return x;
}

Vector6d qd_d(double t, Vector6d q_des){
	Vector6d qd_des;
	qd_des << 0, 0, 0, 0.1, 0, 0;
	return qd_des;
}

Vector6d qdd_d(double t, Vector6d q_des, Vector6d qd_des){
	Vector6d qdd_des;
	qdd_des << MatrixXd::Zero(6,1);
	return qdd_des;
}

MatrixXd Y(Matrix3d R, Vector6d qd, Vector6d qd_d, Vector6d qdd_d){
	//takes in R (sensor to world), qd (linear and angular rates of sensor in N), qd_des & qdd_des (in N), returns regressor Y
	MatrixXd Yout(6,18);
	Yout << (R(1,0)*qdd_d[5] - R(2,0)*qdd_d[4] + qd_d[4]*(R(0,0)*qd[4] - R(1,0)*qd[3]) + qd_d[5]*(R(0,0)*qd[5] - R(2,0)*qd[3])), (R(1,1)*qdd_d[5] - R(2,1)*qdd_d[4] + qd_d[4]*(R(0,1)*qd[4] - R(1,1)*qd[3]) + qd_d[5]*(R(0,1)*qd[5] - R(2,1)*qd[3])), (R(1,2)*qdd_d[5] - R(2,2)*qdd_d[4] + qd_d[4]*(R(0,2)*qd[4] - R(1,2)*qd[3]) + qd_d[5]*(R(0,2)*qd[5] - R(2,2)*qd[3])), qdd_d[0], MatrixXd::Zero(1,14),
			(R(2,0)*qdd_d[3] - R(0,0)*qdd_d[5] - qd_d[3]*(R(0,0)*qd[4] - R(1,0)*qd[3]) + qd_d[5]*(R(1,0)*qd[5] - R(2,0)*qd[4])), (R(2,1)*qdd_d[3] - R(0,1)*qdd_d[5] - qd_d[3]*(R(0,1)*qd[4] - R(1,1)*qd[3]) + qd_d[5]*(R(1,1)*qd[5] - R(2,1)*qd[4])), (R(2,2)*qdd_d[3] - R(0,2)*qdd_d[5] - qd_d[3]*(R(0,2)*qd[4] - R(1,2)*qd[3]) + qd_d[5]*(R(1,2)*qd[5] - R(2,2)*qd[4])), qdd_d[1], MatrixXd::Zero(1,14),
			(R(0,0)*qdd_d[4] - R(1,0)*qdd_d[3] - qd_d[3]*(R(0,0)*qd[5] - R(2,0)*qd[3]) - qd_d[4]*(R(1,0)*qd[5] - R(2,0)*qd[4])), (R(0,1)*qdd_d[4] - R(1,1)*qdd_d[3] - qd_d[3]*(R(0,1)*qd[5] - R(2,1)*qd[3]) - qd_d[4]*(R(1,1)*qd[5] - R(2,1)*qd[4])), (R(0,2)*qdd_d[4] - R(1,2)*qdd_d[3] - qd_d[3]*(R(0,2)*qd[5] - R(2,2)*qd[3]) - qd_d[4]*(R(1,2)*qd[5] - R(2,2)*qd[4])), qdd_d[2], MatrixXd::Zero(1,14),
			-(R(1,0)*qdd_d[2] - R(2,0)*qdd_d[1]), -(R(1,1)*qdd_d[2] - R(2,1)*qdd_d[1]), -(R(1,2)*qdd_d[2] - R(2,2)*qdd_d[1]), 0, (qdd_d[4]*(R(0,1)*R(1,1) + R(0,2)*R(1,2)) + qdd_d[5]*(R(0,1)*R(2,1) + R(0,2)*R(2,2)) + qdd_d[3]*(pow(R(0,1),2 + pow(R(0,2),2)) - qd_d[3]*(R(0,1)*(R(1,1)*qd[5] - R(2,1)*qd[4]) + R(0,2)*(R(1,2)*qd[5] - R(2,2)*qd[4])) - qd_d[4]*(R(1,1)*(R(1,1)*qd[5] - R(2,1)*qd[4]) + R(1,2)*(R(1,2)*qd[5] - R(2,2)*qd[4])) - qd_d[5]*(R(2,1)*(R(1,1)*qd[5] - R(2,1)*qd[4]) + R(2,2)*(R(1,2)*qd[5] - R(2,2)*qd[4]))), (qd_d[3]*(R(0,1)*(R(1,0)*qd[5] - R(2,0)*qd[4]) + R(0,0)*(R(1,1)*qd[5] - R(2,1)*qd[4])) - qdd_d[5]*(R(0,0)*R(2,1) + R(0,1)*R(2,0)) - qdd_d[4]*(R(0,0)*R(1,1) + R(0,1)*R(1,0)) + qd_d[4]*(R(1,1)*(R(1,0)*qd[5] - R(2,0)*qd[4]) + R(1,0)*(R(1,1)*qd[5] - R(2,1)*qd[4])) + qd_d[5]*(R(2,1)*(R(1,0)*qd[5] - R(2,0)*qd[4]) + R(2,0)*(R(1,1)*qd[5] - R(2,1)*qd[4])) - 2*R(0,0)*R(0,1)*qdd_d[3]), (qd_d[3]*(R(0,2)*(R(1,0)*qd[5] - R(2,0)*qd[4]) + R(0,0)*(R(1,2)*qd[5] - R(2,2)*qd[4])) - qdd_d[5]*(R(0,0)*R(2,2) + R(0,2)*R(2,0)) - qdd_d[4]*(R(0,0)*R(1,2) + R(0,2)*R(1,0)) + qd_d[4]*(R(1,2)*(R(1,0)*qd[5] - R(2,0)*qd[4]) + R(1,0)*(R(1,2)*qd[5] - R(2,2)*qd[4])) + qd_d[5]*(R(2,2)*(R(1,0)*qd[5] - R(2,0)*qd[4]) + R(2,0)*(R(1,2)*qd[5] - R(2,2)*qd[4])) - 2*R(0,0)*R(0,2)*qdd_d[3]), (qdd_d[4]*(R(0,0)*R(1,0) + R(0,2)*R(1,2)) + qdd_d[5]*(R(0,0)*R(2,0) + R(0,2)*R(2,2)) + qdd_d[3]*(pow(R(0,0),2) + pow(R(0,2),2)) - qd_d[3]*(R(0,0)*(R(1,0)*qd[5] - R(2,0)*qd[4]) + R(0,2)*(R(1,2)*qd[5] - R(2,2)*qd[4])) - qd_d[4]*(R(1,0)*(R(1,0)*qd[5] - R(2,0)*qd[4]) + R(1,2)*(R(1,2)*qd[5] - R(2,2)*qd[4])) - qd_d[5]*(R(2,0)*(R(1,0)*qd[5] - R(2,0)*qd[4]) + R(2,2)*(R(1,2)*qd[5] - R(2,2)*qd[4]))), (qd_d[3]*(R(0,2)*(R(1,1)*qd[5] - R(2,1)*qd[4]) + R(0,1)*(R(1,2)*qd[5] - R(2,2)*qd[4])) - qdd_d[5]*(R(0,1)*R(2,2) + R(0,2)*R(2,1)) - qdd_d[4]*(R(0,1)*R(1,2) + R(0,2)*R(1,1)) + qd_d[4]*(R(1,2)*(R(1,1)*qd[5] - R(2,1)*qd[4]) + R(1,1)*(R(1,2)*qd[5] - R(2,2)*qd[4])) + qd_d[5]*(R(2,2)*(R(1,1)*qd[5] - R(2,1)*qd[4]) + R(2,1)*(R(1,2)*qd[5] - R(2,2)*qd[4])) - 2*R(0,1)*R(0,2)*qdd_d[3]), (qdd_d[4]*(R(0,0)*R(1,0) + R(0,1)*R(1,1)) + qdd_d[5]*(R(0,0)*R(2,0) + R(0,1)*R(2,1)) + qdd_d[3]*(pow(R(0,0),2) + pow(R(0,1),2)) - qd_d[3]*(R(0,0)*(R(1,0)*qd[5] - R(2,0)*qd[4]) + R(0,1)*(R(1,1)*qd[5] - R(2,1)*qd[4])) - qd_d[4]*(R(1,0)*(R(1,0)*qd[5] - R(2,0)*qd[4]) + R(1,1)*(R(1,1)*qd[5] - R(2,1)*qd[4])) - qd_d[5]*(R(2,0)*(R(1,0)*qd[5] - R(2,0)*qd[4]) + R(2,1)*(R(1,1)*qd[5] - R(2,1)*qd[4]))), (qdd_d[4]*(R(0,0)*R(1,1) + R(0,1)*R(1,0)) + qdd_d[5]*(R(0,0)*R(2,1) + R(0,1)*R(2,0)) - qd_d[3]*(R(0,1)*(R(1,0)*qd[5] - R(2,0)*qd[4]) + R(0,0)*(R(1,1)*qd[5] - R(2,1)*qd[4])) - qd_d[4]*(R(1,1)*(R(1,0)*qd[5] - R(2,0)*qd[4]) + R(1,0)*(R(1,1)*qd[5] - R(2,1)*qd[4])) - qd_d[5]*(R(2,1)*(R(1,0)*qd[5] - R(2,0)*qd[4]) + R(2,0)*(R(1,1)*qd[5] - R(2,1)*qd[4])) + 2*R(0,0)*R(0,1)*qdd_d[3]), (qdd_d[4]*(R(0,0)*R(1,2) + R(0,2)*R(1,0)) + qdd_d[5]*(R(0,0)*R(2,2) + R(0,2)*R(2,0)) - qd_d[3]*(R(0,2)*(R(1,0)*qd[5] - R(2,0)*qd[4]) + R(0,0)*(R(1,2)*qd[5] - R(2,2)*qd[4])) - qd_d[4]*(R(1,2)*(R(1,0)*qd[5] - R(2,0)*qd[4]) + R(1,0)*(R(1,2)*qd[5] - R(2,2)*qd[4])) - qd_d[5]*(R(2,2)*(R(1,0)*qd[5] - R(2,0)*qd[4]) + R(2,0)*(R(1,2)*qd[5] - R(2,2)*qd[4])) + 2*R(0,0)*R(0,2)*qdd_d[3]), (qdd_d[4]*(R(0,1)*R(1,2) + R(0,2)*R(1,1)) + qdd_d[5]*(R(0,1)*R(2,2) + R(0,2)*R(2,1)) - qd_d[3]*(R(0,2)*(R(1,1)*qd[5] - R(2,1)*qd[4]) + R(0,1)*(R(1,2)*qd[5] - R(2,2)*qd[4])) - qd_d[4]*(R(1,2)*(R(1,1)*qd[5] - R(2,1)*qd[4]) + R(1,1)*(R(1,2)*qd[5] - R(2,2)*qd[4])) - qd_d[5]*(R(2,2)*(R(1,1)*qd[5] - R(2,1)*qd[4]) + R(2,1)*(R(1,2)*qd[5] - R(2,2)*qd[4])) + 2*R(0,1)*R(0,2)*qdd_d[3]), (pow(R(0,0),2)*qdd_d[3] + R(0,0)*R(1,0)*qdd_d[4] + R(0,0)*R(2,0)*qdd_d[5] - R(0,0)*qd_d[3]*(R(1,0)*qd[5] - R(2,0)*qd[4]) - R(1,0)*qd_d[4]*(R(1,0)*qd[5] - R(2,0)*qd[4]) - R(2,0)*qd_d[5]*(R(1,0)*qd[5] - R(2,0)*qd[4])), (pow(R(0,1),2)*qdd_d[3] + R(0,1)*R(1,1)*qdd_d[4] + R(0,1)*R(2,1)*qdd_d[5] - R(0,1)*qd_d[3]*(R(1,1)*qd[5] - R(2,1)*qd[4]) - R(1,1)*qd_d[4]*(R(1,1)*qd[5] - R(2,1)*qd[4]) - R(2,1)*qd_d[5]*(R(1,1)*qd[5] - R(2,1)*qd[4])), (pow(R(0,2),2)*qdd_d[3] + R(0,2)*R(1,2)*qdd_d[4] + R(0,2)*R(2,2)*qdd_d[5] - R(0,2)*qd_d[3]*(R(1,2)*qd[5] - R(2,2)*qd[4]) - R(1,2)*qd_d[4]*(R(1,2)*qd[5] - R(2,2)*qd[4]) - R(2,2)*qd_d[5]*(R(1,2)*qd[5] - R(2,2)*qd[4])), MatrixXd::Zero(1,3),
			(R(0,0)*qdd_d[2] - R(2,0)*qdd_d[0]), (R(0,1)*qdd_d[2] - R(2,1)*qdd_d[0]), (R(0,2)*qdd_d[2] - R(2,2)*qdd_d[0]), 0, (qdd_d[3]*(R(0,1)*R(1,1) + R(0,2)*R(1,2)) + qdd_d[5]*(R(1,1)*R(2,1) + R(1,2)*R(2,2)) + qdd_d[4]*(pow(R(1,1),2) + pow(R(1,2),2)) + qd_d[3]*(R(0,1)*(R(0,1)*qd[5] - R(2,1)*qd[3]) + R(0,2)*(R(0,2)*qd[5] - R(2,2)*qd[3])) + qd_d[4]*(R(1,1)*(R(0,1)*qd[5] - R(2,1)*qd[3]) + R(1,2)*(R(0,2)*qd[5] - R(2,2)*qd[3])) + qd_d[5]*(R(2,1)*(R(0,1)*qd[5] - R(2,1)*qd[3]) + R(2,2)*(R(0,2)*qd[5] - R(2,2)*qd[3]))), (- qdd_d[3]*(R(0,0)*R(1,1) + R(0,1)*R(1,0)) - qdd_d[5]*(R(1,0)*R(2,1) + R(1,1)*R(2,0)) - qd_d[3]*(R(0,1)*(R(0,0)*qd[5] - R(2,0)*qd[3]) + R(0,0)*(R(0,1)*qd[5] - R(2,1)*qd[3])) - qd_d[4]*(R(1,1)*(R(0,0)*qd[5] - R(2,0)*qd[3]) + R(1,0)*(R(0,1)*qd[5] - R(2,1)*qd[3])) - qd_d[5]*(R(2,1)*(R(0,0)*qd[5] - R(2,0)*qd[3]) + R(2,0)*(R(0,1)*qd[5] - R(2,1)*qd[3])) - 2*R(1,0)*R(1,1)*qdd_d[4]), (- qdd_d[3]*(R(0,0)*R(1,2) + R(0,2)*R(1,0)) - qdd_d[5]*(R(1,0)*R(2,2) + R(1,2)*R(2,0)) - qd_d[3]*(R(0,2)*(R(0,0)*qd[5] - R(2,0)*qd[3]) + R(0,0)*(R(0,2)*qd[5] - R(2,2)*qd[3])) - qd_d[4]*(R(1,2)*(R(0,0)*qd[5] - R(2,0)*qd[3]) + R(1,0)*(R(0,2)*qd[5] - R(2,2)*qd[3])) - qd_d[5]*(R(2,2)*(R(0,0)*qd[5] - R(2,0)*qd[3]) + R(2,0)*(R(0,2)*qd[5] - R(2,2)*qd[3])) - 2*R(1,0)*R(1,2)*qdd_d[4]), (qdd_d[3]*(R(0,0)*R(1,0) + R(0,2)*R(1,2)) + qdd_d[5]*(R(1,0)*R(2,0) + R(1,2)*R(2,2)) + qdd_d[4]*(pow(R(1,0),2) + pow(R(1,2),2)) + qd_d[3]*(R(0,0)*(R(0,0)*qd[5] - R(2,0)*qd[3]) + R(0,2)*(R(0,2)*qd[5] - R(2,2)*qd[3])) + qd_d[4]*(R(1,0)*(R(0,0)*qd[5] - R(2,0)*qd[3]) + R(1,2)*(R(0,2)*qd[5] - R(2,2)*qd[3])) + qd_d[5]*(R(2,0)*(R(0,0)*qd[5] - R(2,0)*qd[3]) + R(2,2)*(R(0,2)*qd[5] - R(2,2)*qd[3]))), (- qdd_d[3]*(R(0,1)*R(1,2) + R(0,2)*R(1,1)) - qdd_d[5]*(R(1,1)*R(2,2) + R(1,2)*R(2,1)) - qd_d[3]*(R(0,2)*(R(0,1)*qd[5] - R(2,1)*qd[3]) + R(0,1)*(R(0,2)*qd[5] - R(2,2)*qd[3])) - qd_d[4]*(R(1,2)*(R(0,1)*qd[5] - R(2,1)*qd[3]) + R(1,1)*(R(0,2)*qd[5] - R(2,2)*qd[3])) - qd_d[5]*(R(2,2)*(R(0,1)*qd[5] - R(2,1)*qd[3]) + R(2,1)*(R(0,2)*qd[5] - R(2,2)*qd[3])) - 2*R(1,1)*R(1,2)*qdd_d[4]), (qdd_d[3]*(R(0,0)*R(1,0) + R(0,1)*R(1,1)) + qdd_d[5]*(R(1,0)*R(2,0) + R(1,1)*R(2,1)) + qdd_d[4]*(pow(R(1,0),2) + pow(R(1,1),2)) + qd_d[3]*(R(0,0)*(R(0,0)*qd[5] - R(2,0)*qd[3]) + R(0,1)*(R(0,1)*qd[5] - R(2,1)*qd[3])) + qd_d[4]*(R(1,0)*(R(0,0)*qd[5] - R(2,0)*qd[3]) + R(1,1)*(R(0,1)*qd[5] - R(2,1)*qd[3])) + qd_d[5]*(R(2,0)*(R(0,0)*qd[5] - R(2,0)*qd[3]) + R(2,1)*(R(0,1)*qd[5] - R(2,1)*qd[3]))), (qdd_d[3]*(R(0,0)*R(1,1) + R(0,1)*R(1,0)) + qdd_d[5]*(R(1,0)*R(2,1) + R(1,1)*R(2,0)) + qd_d[3]*(R(0,1)*(R(0,0)*qd[5] - R(2,0)*qd[3]) + R(0,0)*(R(0,1)*qd[5] - R(2,1)*qd[3])) + qd_d[4]*(R(1,1)*(R(0,0)*qd[5] - R(2,0)*qd[3]) + R(1,0)*(R(0,1)*qd[5] - R(2,1)*qd[3])) + qd_d[5]*(R(2,1)*(R(0,0)*qd[5] - R(2,0)*qd[3]) + R(2,0)*(R(0,1)*qd[5] - R(2,1)*qd[3])) + 2*R(1,0)*R(1,1)*qdd_d[4]), (qdd_d[3]*(R(0,0)*R(1,2) + R(0,2)*R(1,0)) + qdd_d[5]*(R(1,0)*R(2,2) + R(1,2)*R(2,0)) + qd_d[3]*(R(0,2)*(R(0,0)*qd[5] - R(2,0)*qd[3]) + R(0,0)*(R(0,2)*qd[5] - R(2,2)*qd[3])) + qd_d[4]*(R(1,2)*(R(0,0)*qd[5] - R(2,0)*qd[3]) + R(1,0)*(R(0,2)*qd[5] - R(2,2)*qd[3])) + qd_d[5]*(R(2,2)*(R(0,0)*qd[5] - R(2,0)*qd[3]) + R(2,0)*(R(0,2)*qd[5] - R(2,2)*qd[3])) + 2*R(1,0)*R(1,2)*qdd_d[4]), (qdd_d[3]*(R(0,1)*R(1,2) + R(0,2)*R(1,1)) + qdd_d[5]*(R(1,1)*R(2,2) + R(1,2)*R(2,1)) + qd_d[3]*(R(0,2)*(R(0,1)*qd[5] - R(2,1)*qd[3]) + R(0,1)*(R(0,2)*qd[5] - R(2,2)*qd[3])) + qd_d[4]*(R(1,2)*(R(0,1)*qd[5] - R(2,1)*qd[3]) + R(1,1)*(R(0,2)*qd[5] - R(2,2)*qd[3])) + qd_d[5]*(R(2,2)*(R(0,1)*qd[5] - R(2,1)*qd[3]) + R(2,1)*(R(0,2)*qd[5] - R(2,2)*qd[3])) + 2*R(1,1)*R(1,2)*qdd_d[4]), (pow(R(1,0),2)*qdd_d[4] + R(0,0)*R(1,0)*qdd_d[3] + R(1,0)*R(2,0)*qdd_d[5] + R(0,0)*qd_d[3]*(R(0,0)*qd[5] - R(2,0)*qd[3]) + R(1,0)*qd_d[4]*(R(0,0)*qd[5] - R(2,0)*qd[3]) + R(2,0)*qd_d[5]*(R(0,0)*qd[5] - R(2,0)*qd[3])), (pow(R(1,1),2)*qdd_d[4] + R(0,1)*R(1,1)*qdd_d[3] + R(1,1)*R(2,1)*qdd_d[5] + R(0,1)*qd_d[3]*(R(0,1)*qd[5] - R(2,1)*qd[3]) + R(1,1)*qd_d[4]*(R(0,1)*qd[5] - R(2,1)*qd[3]) + R(2,1)*qd_d[5]*(R(0,1)*qd[5] - R(2,1)*qd[3])), (pow(R(1,2),2)*qdd_d[4] + R(0,2)*R(1,2)*qdd_d[3] + R(1,2)*R(2,2)*qdd_d[5] + R(0,2)*qd_d[3]*(R(0,2)*qd[5] - R(2,2)*qd[3]) + R(1,2)*qd_d[4]*(R(0,2)*qd[5] - R(2,2)*qd[3]) + R(2,2)*qd_d[5]*(R(0,2)*qd[5] - R(2,2)*qd[3])), MatrixXd::Zero(1,3),
			-(R(0,0)*qdd_d[1] - R(1,0)*qdd_d[0]), -(R(0,1)*qdd_d[1] - R(1,1)*qdd_d[0]), -(R(0,2)*qdd_d[1] - R(1,2)*qdd_d[0]), 0, (qdd_d[3]*(R(0,1)*R(2,1) + R(0,2)*R(2,2)) + qdd_d[4]*(R(1,1)*R(2,1) + R(1,2)*R(2,2)) + qdd_d[5]*(pow(R(2,1),2) + pow(R(2,2),2)) - qd_d[3]*(R(0,1)*(R(0,1)*qd[4] - R(1,1)*qd[3]) + R(0,2)*(R(0,2)*qd[4] - R(1,2)*qd[3])) - qd_d[4]*(R(1,1)*(R(0,1)*qd[4] - R(1,1)*qd[3]) + R(1,2)*(R(0,2)*qd[4] - R(1,2)*qd[3])) - qd_d[5]*(R(2,1)*(R(0,1)*qd[4] - R(1,1)*qd[3]) + R(2,2)*(R(0,2)*qd[4] - R(1,2)*qd[3]))), (qd_d[3]*(R(0,1)*(R(0,0)*qd[4] - R(1,0)*qd[3]) + R(0,0)*(R(0,1)*qd[4] - R(1,1)*qd[3])) - qdd_d[4]*(R(1,0)*R(2,1) + R(1,1)*R(2,0)) - qdd_d[3]*(R(0,0)*R(2,1) + R(0,1)*R(2,0)) + qd_d[4]*(R(1,1)*(R(0,0)*qd[4] - R(1,0)*qd[3]) + R(1,0)*(R(0,1)*qd[4] - R(1,1)*qd[3])) + qd_d[5]*(R(2,1)*(R(0,0)*qd[4] - R(1,0)*qd[3]) + R(2,0)*(R(0,1)*qd[4] - R(1,1)*qd[3])) - 2*R(2,0)*R(2,1)*qdd_d[5]), (qd_d[3]*(R(0,2)*(R(0,0)*qd[4] - R(1,0)*qd[3]) + R(0,0)*(R(0,2)*qd[4] - R(1,2)*qd[3])) - qdd_d[4]*(R(1,0)*R(2,2) + R(1,2)*R(2,0)) - qdd_d[3]*(R(0,0)*R(2,2) + R(0,2)*R(2,0)) + qd_d[4]*(R(1,2)*(R(0,0)*qd[4] - R(1,0)*qd[3]) + R(1,0)*(R(0,2)*qd[4] - R(1,2)*qd[3])) + qd_d[5]*(R(2,2)*(R(0,0)*qd[4] - R(1,0)*qd[3]) + R(2,0)*(R(0,2)*qd[4] - R(1,2)*qd[3])) - 2*R(2,0)*R(2,2)*qdd_d[5]), (qdd_d[3]*(R(0,0)*R(2,0) + R(0,2)*R(2,2)) + qdd_d[4]*(R(1,0)*R(2,0) + R(1,2)*R(2,2)) + qdd_d[5]*(pow(R(2,0),2) + pow(R(2,2),2)) - qd_d[3]*(R(0,0)*(R(0,0)*qd[4] - R(1,0)*qd[3]) + R(0,2)*(R(0,2)*qd[4] - R(1,2)*qd[3])) - qd_d[4]*(R(1,0)*(R(0,0)*qd[4] - R(1,0)*qd[3]) + R(1,2)*(R(0,2)*qd[4] - R(1,2)*qd[3])) - qd_d[5]*(R(2,0)*(R(0,0)*qd[4] - R(1,0)*qd[3]) + R(2,2)*(R(0,2)*qd[4] - R(1,2)*qd[3]))),  (qd_d[3]*(R(0,2)*(R(0,1)*qd[4] - R(1,1)*qd[3]) + R(0,1)*(R(0,2)*qd[4] - R(1,2)*qd[3])) - qdd_d[4]*(R(1,1)*R(2,2) + R(1,2)*R(2,1)) - qdd_d[3]*(R(0,1)*R(2,2) + R(0,2)*R(2,1)) + qd_d[4]*(R(1,2)*(R(0,1)*qd[4] - R(1,1)*qd[3]) + R(1,1)*(R(0,2)*qd[4] - R(1,2)*qd[3])) + qd_d[5]*(R(2,2)*(R(0,1)*qd[4] - R(1,1)*qd[3]) + R(2,1)*(R(0,2)*qd[4] - R(1,2)*qd[3])) - 2*R(2,1)*R(2,2)*qdd_d[5]), (qdd_d[3]*(R(0,0)*R(2,0) + R(0,1)*R(2,1)) + qdd_d[4]*(R(1,0)*R(2,0) + R(1,1)*R(2,1)) + qdd_d[5]*(pow(R(2,0),2) + pow(R(2,1),2)) - qd_d[3]*(R(0,0)*(R(0,0)*qd[4] - R(1,0)*qd[3]) + R(0,1)*(R(0,1)*qd[4] - R(1,1)*qd[3])) - qd_d[4]*(R(1,0)*(R(0,0)*qd[4] - R(1,0)*qd[3]) + R(1,1)*(R(0,1)*qd[4] - R(1,1)*qd[3])) - qd_d[5]*(R(2,0)*(R(0,0)*qd[4] - R(1,0)*qd[3]) + R(2,1)*(R(0,1)*qd[4] - R(1,1)*qd[3]))), (qdd_d[3]*(R(0,0)*R(2,1) + R(0,1)*R(2,0)) + qdd_d[4]*(R(1,0)*R(2,1) + R(1,1)*R(2,0)) - qd_d[3]*(R(0,1)*(R(0,0)*qd[4] - R(1,0)*qd[3]) + R(0,0)*(R(0,1)*qd[4] - R(1,1)*qd[3])) - qd_d[4]*(R(1,1)*(R(0,0)*qd[4] - R(1,0)*qd[3]) + R(1,0)*(R(0,1)*qd[4] - R(1,1)*qd[3])) - qd_d[5]*(R(2,1)*(R(0,0)*qd[4] - R(1,0)*qd[3]) + R(2,0)*(R(0,1)*qd[4] - R(1,1)*qd[3])) + 2*R(2,0)*R(2,1)*qdd_d[5]), (qdd_d[3]*(R(0,0)*R(2,2) + R(0,2)*R(2,0)) + qdd_d[4]*(R(1,0)*R(2,2) + R(1,2)*R(2,0)) - qd_d[3]*(R(0,2)*(R(0,0)*qd[4] - R(1,0)*qd[3]) + R(0,0)*(R(0,2)*qd[4] - R(1,2)*qd[3])) - qd_d[4]*(R(1,2)*(R(0,0)*qd[4] - R(1,0)*qd[3]) + R(1,0)*(R(0,2)*qd[4] - R(1,2)*qd[3])) - qd_d[5]*(R(2,2)*(R(0,0)*qd[4] - R(1,0)*qd[3]) + R(2,0)*(R(0,2)*qd[4] - R(1,2)*qd[3])) + 2*R(2,0)*R(2,2)*qdd_d[5]), (qdd_d[3]*(R(0,1)*R(2,2) + R(0,2)*R(2,1)) + qdd_d[4]*(R(1,1)*R(2,2) + R(1,2)*R(2,1)) - qd_d[3]*(R(0,2)*(R(0,1)*qd[4] - R(1,1)*qd[3]) + R(0,1)*(R(0,2)*qd[4] - R(1,2)*qd[3])) - qd_d[4]*(R(1,2)*(R(0,1)*qd[4] - R(1,1)*qd[3]) + R(1,1)*(R(0,2)*qd[4] - R(1,2)*qd[3])) - qd_d[5]*(R(2,2)*(R(0,1)*qd[4] - R(1,1)*qd[3]) + R(2,1)*(R(0,2)*qd[4] - R(1,2)*qd[3])) + 2*R(2,1)*R(2,2)*qdd_d[5]), (pow(R(2,0),2)*qdd_d[5] + R(0,0)*R(2,0)*qdd_d[3] + R(1,0)*R(2,0)*qdd_d[4] - R(0,0)*qd_d[3]*(R(0,0)*qd[4] - R(1,0)*qd[3]) - R(1,0)*qd_d[4]*(R(0,0)*qd[4] - R(1,0)*qd[3]) - R(2,0)*qd_d[5]*(R(0,0)*qd[4] - R(1,0)*qd[3])), (pow(R(2,1),2)*qdd_d[5] + R(0,1)*R(2,1)*qdd_d[3] + R(1,1)*R(2,1)*qdd_d[4] - R(0,1)*qd_d[3]*(R(0,1)*qd[4] - R(1,1)*qd[3]) - R(1,1)*qd_d[4]*(R(0,1)*qd[4] - R(1,1)*qd[3]) - R(2,1)*qd_d[5]*(R(0,1)*qd[4] - R(1,1)*qd[3])), (pow(R(2,2),2)*qdd_d[5] + R(0,2)*R(2,2)*qdd_d[3] + R(1,2)*R(2,2)*qdd_d[4] - R(0,2)*qd_d[3]*(R(0,2)*qd[4] - R(1,2)*qd[3]) - R(1,2)*qd_d[4]*(R(0,2)*qd[4] - R(1,2)*qd[3]) - R(2,2)*qd_d[5]*(R(0,2)*qd[4] - R(1,2)*qd[3])), MatrixXd::Zero(1,3);
	return Yout;
}

Matrix6d Minv(Matrix3d R, VectorXd a){
	Vector3d ri;
	ri << a[15], a[16], a[17];
	Matrix6d Mout;
	Mout << MatrixXd::Identity(3,3), MatrixXd::Zero(3,3),
			-R*cross(ri), MatrixXd::Identity(3,3);
	return Mout;
}

MatrixXd Z(Matrix3d R, Vector6d F){
	//takes in R (sensor to world) and F (in N), returns regressor Z
	MatrixXd Zout(6,18);
	Zout << MatrixXd::Zero(3,18),
			MatrixXd::Zero(1,15), (F[1]*R(2,0) - F[2]*R(1,0)), (F[1]*R(2,1) - F[2]*R(1,1)), (F[1]*R(2,2) - F[2]*R(1,2)),
			MatrixXd::Zero(1,15), (F[2]*R(0,0) - F[0]*R(2,0)), (F[2]*R(0,1) - F[0]*R(2,1)), (F[2]*R(0,2) - F[0]*R(2,2)),
			MatrixXd::Zero(1,15), (F[0]*R(1,0) - F[1]*R(0,0)), (F[0]*R(1,1) - F[1]*R(0,1)), (F[0]*R(1,2) - F[1]*R(0,2));
	return Zout;
}


int main(int argc, char *argv[]){
	std::stringstream ss;
	ss << "controller" << argv[1];
	ros::init(argc,argv,ss.str());
	ros::NodeHandle n;
	ros::Duration(2).sleep(); 
	Controller ctrl (atoi(argv[1]),n);
	ROS_INFO("all good!");
	ros::Subscriber stateSub = n.subscribe("/gazebo/link_states",1, &Controller::stateCallback, &ctrl);
	ros::spin();
	return 0;
}