#ifndef MODEL_PUSH_CC
#define MODEL_PUSH_CC

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/math/gzmath.hh>


namespace gazebo
{
  /// \brief A plugin to control an hexapod.
  class RhexPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: RhexPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: 
	virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Safety check
        if (_model->GetJointCount() == 0)
        {
            std::cerr << "Invalid joint count, Pexapod plugin not loaded\n";
        return;
        }

namespace gazebo
{
      class RhexPlugin : public ModelPlugin
    {
	/// \brief Constructor
	public: RhexPlugin() {}

	    /// \brief The load function is called by Gazebo when the plugin is
	    /// inserted into simulation
	    /// \param[in] _model A pointer to the model that this plugin is
	    /// attached to.
	    /// \param[in] _sdf A pointer to the plugin's SDF element.
        public:
            void Load( physics::ModelPtr _parent, sdf::ElementPtr );
            /*
            void testFoo(double x, double y, double z) {
                std::cout << "foo: x="<< x << ",y=" << y << ",z=" << z << std::endl;
            }
            //*/
            void OnUpdate( const common::UpdateInfo );
            
        private:

            physics::ModelPtr model;
            physics::JointPtr joint1;
            physics::JointPtr joint2;
            physics::JointPtr joint3;
            physics::JointPtr joint4;
            physics::JointPtr joint5;
            physics::JointPtr joint6;
            physics::JointPtr jointBody;
            physics::JointController * controller;
            event::ConnectionPtr updateConnection;

            void drive135(double, int);
            void drive246(double, int);

            void controller_timer(void);
            void controller_angle1(void);
            void controller_angle2(void);

            transport::NodePtr node;
            gazebo::transport::PublisherPtr pub;
    
            void getCenterOfMass(void);
            

    
	    /// \brief A node used for transport
	    private: transport::NodePtr node;
	    
	    /// \brief A subscriber to a named topic.
	    private: transport::SubscriberPtr sub;
	    
	    /// \brief Pointer to the model.
	    private: physics::ModelPtr model;
	    
	    /// \brief Pointer to the joint.
	    // Leg 1
	    private: physics::JointPtr joint_leg1;
	    // Leg 2
	    private: physics::JointPtr joint_leg2;
	    // Leg 3
	    private: physics::JointPtr joint_leg3;
	    // leg 4 
	    private: physics::JointPtr joint_leg4;
	    // Leg 5
	    private: physics::JointPtr joint_leg5;
	    // Leg 6
	    private: physics::JointPtr joint_leg6;
	    
	    /// \brief A PID controller for the joint.
	    // Leg 1
	    private: common::PID pid_leg1;
	    // Leg 2
	    private: common::PID pid_leg2;
	    // Leg 3
	    private: common::PID pid_leg3;
	    // Leg 4
	    private: common::PID pid_leg4;
	    // Leg 5
	    private: common::PID pid_leg5;
	    // Leg 6
	    private: common::PID pid_leg6;
	  };

	  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
	  GZ_REGISTER_MODEL_PLUGIN(RhexPlugin)
	  

    };

}


#endif
