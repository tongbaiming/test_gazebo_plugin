#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <string>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class VelodynePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: VelodynePlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Just output a message for now
      std::cerr << "\nfuck *****************************The lidar test plugin is attach to model[" <<
        _model->GetName() << "]\n";
			// Safety check
  if (_model->GetJointCount() == 0)
  {
    std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
    return;
  }

  // Store the model pointer for convenience.
  this->model = _model;
  this->sdf = _sdf;

	 if (!this->sdf->HasElement("jointName"))
  {
    std::cerr << "lidar joint plugin missing <jointName>, defaults to nothing" << std::endl;
    this->joint_name_ = "";
    return ;
  }
	this->joint_name_ = this->sdf->Get<std::string>("jointName");
	std::cerr << "I am in lidar_test_plugin_2, I get joinName: " << this->joint_name_ << std::endl;
  // Get the first joint. We are making an assumption about the model
  // having one joint that is the rotational joint.
  this->joints = _model->GetJoints();
  for (auto i = this->joints.begin(); i != this->joints.end(); i++)
  {
  	std::string::size_type idx = ((*(*i)).GetName()).find(this->joint_name_);
  	std::cerr << "I find the joint: "<<(*(*i)).GetName() << std::endl;
  	if(idx != std::string::npos)
  	{	
  			this->joint = (*i);
  			std::cerr << "idx != npos  " << "I find the right joint: " << this->joint->GetScopedName() << std::endl;
  			std::cerr << "I find the joint: " << this->joint->GetName() << std::endl;
  	}
  }
  this->links = _model->GetLinks();
  for( auto i = this->links.begin(); i != this->links.end(); i++)
  {
  	std::cerr << "I find the link: " << (*(*i)).GetName() << std::endl;
  }
	
  // Setup a P-controller, with a gain of 0.1.
  this->pid = common::PID(0.01, 0, 0);
	std::cerr << this->joint->GetScopedName() << "\n" ;
  // Apply the P-controller to the joint.
  this->model->GetJointController()->SetVelocityPID(
      this->joint->GetScopedName(), this->pid);

  // Set the joint's target velocity. This target velocity is just
  // for demonstration purposes.
  this->velocity = 3.0;
  if (_sdf->HasElement("velocity"))
  this->velocity = _sdf->Get<double>("velocity");
  
  this->model->GetJointController()->SetVelocityTarget(
      this->joint->GetScopedName(), this->velocity); 
      
    }
	private: physics::ModelPtr model;
	private: physics::JointPtr joint;
	private: common::PID pid;
	private: std::string joint_name_;
	private: sdf::ElementPtr sdf;
	private: physics::Joint_V joints;
	private: physics::Link_V links;
	private: double velocity;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif

