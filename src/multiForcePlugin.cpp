/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 *  \author Dave Coleman
 *  \desc   Example ROS plugin for Gazebo
 */

#include <gazebo_plugins/gazebo_ros_force.h>
#include <vector>
#include <sstream>


namespace gazebo
{
// Register this plugin with the simulator

class ForcePlugin : public ModelPlugin
{
	public:
		ForcePlugin();
		~ForcePlugin();
		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
		void commandCallback(const geometry_msgs::Wrench::ConstPtr& std_msgs, int i);
		void UpdateChild();
		void setup();

	private: physics::ModelPtr _model;

	private: ros::NodeHandle* rosnode_;

	private: std::vector<ros::Subscriber> subs;

	private: std::string link_name_;

	private: physics::LinkPtr link_;

  	//container for the wrench that each agent applies to the body
	private: std::vector<geometry_msgs::Wrench> wrench_msgs;

	private: std::vector<gazebo::math::Vector3> offsets;

	private: event::ConnectionPtr update_connection_;

	private: boost::mutex lock_;

	private: int numRobots;
};

ForcePlugin::ForcePlugin()
{
}

ForcePlugin::~ForcePlugin()
{
	event::Events::DisconnectWorldUpdateBegin(this->update_connection_);

		this->rosnode_->shutdown();

	delete this->rosnode_;
}

void ForcePlugin::setup()
{
	for (int i = 0; i < this->numRobots; i++)
	{
		geometry_msgs::Wrench msg;
		msg.force.x = 0;
  		msg.force.y = 0;
  		msg.force.z = 0;
  		msg.torque.x = 0;
  		msg.torque.y = 0;
		msg.torque.z = 0;
		this->wrench_msgs.push_back(msg);
	}
}



void ForcePlugin::commandCallback(const geometry_msgs::Wrench::ConstPtr& _msg, int i)
{
	this->wrench_msgs[i].force.x = _msg->force.x;
  	this->wrench_msgs[i].force.y = _msg->force.y;
  	this->wrench_msgs[i].force.z = _msg->force.z;
  	this->wrench_msgs[i].torque.x = _msg->torque.x;
  	this->wrench_msgs[i].torque.y = _msg->torque.y;
	this->wrench_msgs[i].torque.z = _msg->torque.z;
	//ROS_INFO("Called back.");
}

void ForcePlugin::UpdateChild()
{
	for (int i = 0; i < this->numRobots; i++){
		this->lock_.lock();
		math::Vector3 force(this->wrench_msgs[i].force.x,this->wrench_msgs[i].force.y,this->wrench_msgs[i].force.z);
  		math::Vector3 torque(this->wrench_msgs[i].torque.x,this->wrench_msgs[i].torque.y,this->wrench_msgs[i].torque.z);
  		this->link_->AddForceAtRelativePosition(force,this->offsets[i]);
  		//this->link_->AddRelativeForce(force);
  		this->link_->AddRelativeTorque(torque);
  		this->lock_.unlock();
	}
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void ForcePlugin::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  // Make sure the ROS node for Gazebo has already been initalized
  if (!ros::isInitialized())
	{
  		int argc = 0;
  		char **argv = NULL;
  		ros::init(argc, argv, "gazebo_client",
      	ros::init_options::NoSigintHandler);
	}

	//store _parent in model
	this->_model = _parent;


	this->link_ = this->_model->GetLinks()[0];

	this->numRobots = this->_model->GetLinks().size()-1;

	//this->_model->SetWorldTwist(math::Vector3(0, 0, 0),math::Vector3(3,3,3));

	if (!this->link_)
	  {
	    ROS_FATAL_NAMED("force", "plugin error: link named: %s does not exist\n",this->link_name_.c_str());
	    return;
	}

	this->rosnode_ = new ros::NodeHandle("wrenchPlugin");
	//set up 
	for (int i = 0; i < numRobots; i++)
	{
		std::stringstream topicName;
		topicName << "wrenchCommands" << i;
		ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Wrench>(topicName.str(),1,
   		boost::bind( &ForcePlugin::commandCallback,this,_1,i),ros::VoidPtr(),NULL);
   		ROS_INFO("Subscriber options built");
   		this->subs.push_back(this->rosnode_->subscribe(so));
   		this->offsets.push_back(_model->GetLinks()[i+1]->GetRelativePose().pos);
   		ROS_INFO("one robot built");
	}

	this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&ForcePlugin::UpdateChild, this));

	this->setup();

	ROS_INFO("all robots built");
}

GZ_REGISTER_MODEL_PLUGIN(ForcePlugin);
}