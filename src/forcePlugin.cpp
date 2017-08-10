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



namespace gazebo
{
// Register this plugin with the simulator

class ForcePlugin : public ModelPlugin
{
	public:
		ForcePlugin();
		~ForcePlugin();
		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
		void commandCallback(const geometry_msgs::Wrench::ConstPtr& std_msgs);
		void QueueThread();
		void UpdateChild();
		void clearWrench();

	private: physics::ModelPtr _model;

	private: ros::NodeHandle* rosnode_;

	private: ros::Subscriber sub_;

	private: std::string link_name_;

	private: physics::LinkPtr link_;

	// Custom Callback Queue
  	private: ros::CallbackQueue queue_;
  /// \brief Thead object for the running callback Thread.
  	private: boost::thread callback_queue_thread_;
  /// \brief Container for the wrench force that this plugin exerts on the body.
	private: geometry_msgs::Wrench wrench_msg_;

	private: event::ConnectionPtr update_connection_;

	 private: boost::mutex lock_;
};

ForcePlugin::ForcePlugin()
{
	this->wrench_msg_.force.x = 0;
  	this->wrench_msg_.force.y = 0;
  	this->wrench_msg_.force.z = 0;
  	this->wrench_msg_.torque.x = 0;
  	this->wrench_msg_.torque.y = 0;
	this->wrench_msg_.torque.z = 0;
}

ForcePlugin::~ForcePlugin()
{
	event::Events::DisconnectWorldUpdateBegin(this->update_connection_);

	  // Custom Callback Queue
	  this->queue_.clear();
	  this->queue_.disable();
	  this->rosnode_->shutdown();
	  this->callback_queue_thread_.join();

	delete this->rosnode_;
}



void ForcePlugin::commandCallback(const geometry_msgs::Wrench::ConstPtr& _msg)
{
	 this->wrench_msg_.force.x = _msg->force.x;
  this->wrench_msg_.force.y = _msg->force.y;
  this->wrench_msg_.force.z = _msg->force.z;
  this->wrench_msg_.torque.x = _msg->torque.x;
  this->wrench_msg_.torque.y = _msg->torque.y;
this->wrench_msg_.torque.z = _msg->torque.z;
	//ROS_INFO("Called back.");
}

void ForcePlugin::clearWrench()
{
	this->wrench_msg_.force.x = 0;
  	this->wrench_msg_.force.y = 0;
  	this->wrench_msg_.force.z = 0;
  	this->wrench_msg_.torque.x = 0;
  	this->wrench_msg_.torque.y = 0;
	this->wrench_msg_.torque.z = 0;
}

void ForcePlugin::QueueThread()
{
	static const double timeout = 0.01;

	while (this->rosnode_->ok())
  	{
    	this->queue_.callAvailable(ros::WallDuration(timeout));
	}
}

void ForcePlugin::UpdateChild()
{
	this->lock_.lock();
  	math::Vector3 force(this->wrench_msg_.force.x,this->wrench_msg_.force.y,this->wrench_msg_.force.z);
  	math::Vector3 torque(this->wrench_msg_.torque.x,this->wrench_msg_.torque.y,this->wrench_msg_.torque.z);
  	this->link_->AddRelativeForce(force);
  	this->link_->AddRelativeTorque(torque);
	this->lock_.unlock();
	this->clearWrench();
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

	ROS_INFO("Hello World!");

	//store _parent in model
	this->_model = _parent;


	this->link_ = this->_model->GetLinks()[0];

	if (!this->link_)
	  {
	    ROS_FATAL_NAMED("force", "plugin error: link named: %s does not exist\n",this->link_name_.c_str());
	    return;
	}

	this->rosnode_ = new ros::NodeHandle("wrenchPlugin");


	ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Wrench>(
    "wrenchCommands",1,
    boost::bind( &ForcePlugin::commandCallback,this,_1),
    ros::VoidPtr(), &this->queue_);
	
	this->sub_ = this->rosnode_->subscribe(so);

	this->callback_queue_thread_ = boost::thread( boost::bind( &ForcePlugin::QueueThread,this ) );

	this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&ForcePlugin::UpdateChild, this));

}

GZ_REGISTER_MODEL_PLUGIN(ForcePlugin);
}