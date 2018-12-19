#include <numeric>
#include "model_push.hh"
#include <gazebo/common/common.hh>
#include <cmath>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(RhexPlugin)


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
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Safety check
        if (_model->GetJointCount() == 0)
        {
            std::cerr << "Invalid joint count, Pexapod plugin not loaded\n";
        return;
        }




void ModelPush::Load( physics::ModelPtr _parent, sdf::ElementPtr )
{

        // Safety check
        if (_model->GetJointCount() == 0)
        {
            std::cerr << "Invalid joint count, Pexapod plugin not loaded\n";
        return;
        }
        
        // Store the model pointer for convenience.
        this->model = _model;
        // Leg 1
        this->joint_leg1 = _model->GetJoint("leg1");
        // Leg 2
        this->joint_leg2 = _model->GetJoint("leg2"); 
        // Leg 3
        this->joint_leg3 = _model->GetJoint("leg3");
        // Leg 4
        this->joint_leg4 = _model->GetJoint("leg4");
        // Leg 5
        this->joint_leg5 = _model->GetJoint("leg5");
        // Leg 6
        this->joint_leg6 = _model->GetJoint("leg6");






        // Set lower and upper limit for 6 joints
        // Leg 1
        this->joint_leg1 = _model->SetLowerLimit(0, math::Angle(-0.1));
	      this->joint_leg1 = _model->SetUpperLimit(0, math::Angle( 0.1));		
        // Leg 2
        this->joint_leg2 = _model->SetLowerLimit(0, math::Angle(-0.1)); 
	      this->joint_leg2 = _model->SetUpperLimit(0, math::Angle( 0.1));	
        // Leg 3
        this->joint_leg3 = _model->SetLowerLimit(0, math::Angle(-0.1));
	      this->joint_leg3 = _model->SetUpperLimit(0, math::Angle( 0.1));	
        // Leg 4
        this->joint_leg4 = _model->SetLowerLimit(0, math::Angle(-0.1));
	      this->joint_leg4 = _model->SetUpperLimit(0, math::Angle( 0.1));	
        // Leg 5
        this->joint_leg5 = _model->SetLowerLimit(0, math::Angle(-0.1));
	      this->joint_leg5 = _model->SetUpperLimit(0, math::Angle( 0.1));	
        // Leg 6
        this->joint_leg6 = _model->SetLowerLimit(0, math::Angle(-0.1));
	      this->joint_leg6 = _model->SetUpperLimit(0, math::Angle( 0.1));	
        




        
        // Setup a P-controller, with a gain of 0.1.
        // Leg 1
        this->pid_leg1 = common::PID(0.1, 0, 0);
        // Leg 2
        this->pid_leg2 = common::PID(0.1, 0, 0);
        // Leg 3
        this->pid_leg3 = common::PID(0.1, 0, 0);
        // Leg 4
        this->pid_leg4 = common::PID(0.1, 0, 0);
        // Leg 5
        this->pid_leg5 = common::PID(0.1, 0, 0);
        // Leg 6
        this->pid_leg6 = common::PID(0.1, 0, 0);
        




        // Apply the P-controller to the joint.
        // Leg 1
        this->model->GetJointController()->SetPositionPID(this->joint_leg1->GetScopedName(), this->pid_leg1);
        // Leg 2
        this->model->GetJointController()->SetPositionPID(this->joint_leg1->GetScopedName(), this->pid_leg2);
        // Leg 3
        this->model->GetJointController()->SetPositionPID(this->joint_leg1->GetScopedName(), this->pid_leg3);
        // Leg 4
        this->model->GetJointController()->SetPositionPID(this->joint_leg1->GetScopedName(), this->pid_leg4);
        // Leg 5
        this->model->GetJointController()->SetPositionPID(this->joint_leg1->GetScopedName(), this->pid_leg5);
        // Leg 6
        this->model->GetJointController()->SetPositionPID(this->joint_leg1->GetScopedName(), this->pid_leg6);

        
        this->SetJointPositions(0.0);
    }
    




    /// \brief Set positions of the Pexod Hexapod Legs
    /// \param[in] _position New target position for all the legs
    public: void SetJointPositions(const double &_position)
    {
        // Set the joint's target position.
        std::cout << "Setting Pose All Joints..." << std::endl;
        // Leg 1
        this->model->GetJointController()->SetPositionTarget(this->joint_leg1->GetScopedName(), _position);
        // Leg 2
        this->model->GetJointController()->SetPositionTarget(this->joint_leg2->GetScopedName(), _position);
        // Leg 3
        this->model->GetJointController()->SetPositionTarget(this->joint_leg3->GetScopedName(), _position);
        // Leg 4
        this->model->GetJointController()->SetPositionTarget(this->joint_leg4->GetScopedName(), _position);
        // Leg 5
        this->model->GetJointController()->SetPositionTarget(this->joint_leg5->GetScopedName(), _position);
        // Leg 6
        this->model->GetJointController()->SetPositionTarget(this->joint_leg6->GetScopedName(), _position);
        std::cout << "END Setting Pose  All Joints..." << std::endl;       
    }






    this->controller = new physics::JointController(model);

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&ModelPush::OnUpdate, this, _1));
    //* subscribe to messages
    // Create our node for communication
    this->node = transport::NodePtr(new gazebo::transport::Node());
    node->Init("myNode1");

    // Listen to Gazebo world_stats topic
    this->pub = node->Advertise<gazebo::msgs::Vector3d>("~/myCoG");
    //this->pub->WaitForConnection();

    //*/ end
}


void ModelPush::OnUpdate( const common::UpdateInfo ) 
{
    //this->model->SetLinearVel(math::Vector3(1.03, 0, 0));

    //printf("joint1 angle: %d, %d, %d\n", joint1->GetAngle(0), joint1->GetAngle(1), 
    //        joint1->GetAngle(2));
    static bool startLeg = 0;
    /*
       std::cout << std::setprecision(2) <<  std::fixed << 
       "joint1=" << (abs(this->joint1->GetAngle(0).Degree()) % 360) << ", " <<
       "joint2=" << (abs(this->joint2->GetAngle(0).Degree()) % 360) << ", " <<
       "joint3=" << (abs(this->joint3->GetAngle(0).Degree()) % 360) << ", " << 
       "joint4=" << (abs(this->joint4->GetAngle(0).Degree()) % 360) << ", " <<
       "joint5=" << (abs(this->joint5->GetAngle(0).Degree()) % 360) << ", " << 
       "joint6=" << (abs(this->joint6->GetAngle(0).Degree()) % 360) << ", " <<
       "body joint=" << (abs(this->jointBody->GetAngle(0).Degree()) % 360) << std::endl;
    //*/
    // get tripod gait
    double velocity = 1; // 1 [m/s] 
    //this->controller_timer();
    this->controller_angle1();
    //this->controller_angle2();


    //testFoo();
    //this->model->GetLink("leg1")->
    //this->jointBody->SetAngle(0, math::Angle(0));

    /* experiment with calling contact plugin's public function to get angles.
       static gazebo::ContactPlugin leg1Contact;
       math::Vector3 tmp;
       leg1Contact.gazebo::ContactPlugin::getContactPos(tmp);
       std::cout << "model: " << "x= " << tmp.x << ", y= " << tmp.y  << ", z=" << tmp.z << std::endl;  
    //*/
    return;
}



void ModelPush::controller_timer(void)
{
    static int state = 0, stateTransitionCount = 0;
    static common::Time t0;
    int bound = 5;
    int vel_fast = 5;
    int vel_slow = 2.5;
    int angle = 0;
    common::Time now;
    now.SetToWallTime();
    common::Time timeElapsed;
    double period_fast = 1.0;
    double period_slow = 2.3;

    timeElapsed = (now - t0); 
    switch (state) {
        case 0: // START state
            std::cout << "FSM begins" << std::endl;
            t0.SetToWallTime();
            state = 4;
            std::cout << "entering state 4:" << angle << ",count=" << stateTransitionCount <<  std::endl;
            break;
        case 1:
            this->joint2->SetVelocity(0, vel_fast);
            this->joint4->SetVelocity(0, vel_fast);
            this->joint6->SetVelocity(0, vel_fast);
            this->joint1->SetVelocity(0, 0);
            this->joint3->SetVelocity(0, 0);
            this->joint5->SetVelocity(0, 0);
            angle = abs(this->joint2->GetAngle(0).Degree());
            if (timeElapsed.Double() > period_fast) {
                std::cout << "entering state 2(stop245,slow135):" << angle << ",count=" << stateTransitionCount << std::endl;
                std::cout << "time elapsed=" <<  timeElapsed.Double() << std::endl;
                state = 2;
                stateTransitionCount++; 
                t0.SetToWallTime();
            }
            break;
        case 2:
            this->joint2->SetVelocity(0, 0);  
            this->joint4->SetVelocity(0, 0);  
            this->joint6->SetVelocity(0, 0); 
            this->joint1->SetVelocity(0, vel_slow);
            this->joint3->SetVelocity(0, vel_slow);
            this->joint5->SetVelocity(0, vel_slow);
            angle =  abs(this->joint5->GetAngle(0).Degree());
            if (timeElapsed.Double() > period_slow) {
                std::cout << "entering state 3(stop246,fast135):"<< angle << ",count=" << stateTransitionCount <<  std::endl;
                std::cout << "time elapsed=" << timeElapsed.Double() << std::endl;
                t0.SetToWallTime();
                state = 3;
                stateTransitionCount++; 
            }
            break;
        case 3:
            this->joint1->SetVelocity(0, vel_fast);  
            this->joint3->SetVelocity(0, vel_fast);  
            this->joint5->SetVelocity(0, vel_fast); 
            this->joint2->SetVelocity(0, 0);  
            this->joint4->SetVelocity(0, 0);  
            this->joint6->SetVelocity(0, 0); 
            angle =  abs(this->joint5->GetAngle(0).Degree());
        //if ((angle % 180) <= bound) {
        if (timeElapsed.Double() > period_fast) {
            std::cout << "entering state 4(slow246,stop135):" << angle << ",count=" << stateTransitionCount << std::endl;
            state = 4;
            stateTransitionCount++; 
            t0.SetToWallTime();
            std::cout << "time elapsed=" << timeElapsed.Double() << std::endl;
        }
        break;
    case 4:
        this->joint2->SetVelocity(0, vel_slow);  
        this->joint4->SetVelocity(0, vel_slow);  
        this->joint6->SetVelocity(0, vel_slow); 
        this->joint1->SetVelocity(0, 0);
        this->joint3->SetVelocity(0, 0);
        this->joint5->SetVelocity(0, 0);
        angle =  abs(this->joint2->GetAngle(0).Degree());

        //if ((angle % 180) <= bound) {
        if (timeElapsed.Double() > period_slow) {
            std::cout << "entering state 1(fast246,stop135):" << angle << ",count=" << stateTransitionCount <<  std::endl;
            state = 1;
            stateTransitionCount++; 
            t0.SetToWallTime();
            std::cout << "time elapsed=" << timeElapsed.Double() << std::endl;
        }
        break;

} /* end switch */
return;
}

void ModelPush::controller_angle1(void)
{
    static int state = 0, stateTransitionCount = 0;
    static common::Time t0;
    int bound = 5;
    int vel_fast = 15;
    int vel_slow = 7;
    int angle = 0;
    static bool legSet[7] = {false};

    int angle_stand = 130;
    int angle_switch_speed = 220;

    switch (state) {
        case 0: // START state
            std::cout << "FSM begins" << std::endl;
            state = 4;
            std::cout << "entering state 4:" << angle << ",count=" << stateTransitionCount <<  std::endl;
            break;
        case 1:
            this->joint1->SetVelocity(0, 0);
            this->joint3->SetVelocity(0, 0);
            this->joint5->SetVelocity(0, 0);
            if ((abs(this->joint2->GetAngle(0).Degree()) % 360) == angle_stand) {
                this->joint2->SetVelocity(0, 0);  
                legSet[2] = true;
            } else {
                this->joint2->SetVelocity(0, vel_fast);
            }

            if ((abs(this->joint4->GetAngle(0).Degree()) % 360) == angle_stand) {
                this->joint4->SetVelocity(0, 0);  
                legSet[4] = true;
            } else {
                this->joint4->SetVelocity(0, vel_fast);
            }

            if ((abs(this->joint6->GetAngle(0).Degree()) % 360) == angle_stand) {
                this->joint6->SetVelocity(0, 0);  
                legSet[6] = true;
            } else {
                this->joint6->SetVelocity(0, vel_fast);
            }

            if (legSet[2] && legSet[4] && legSet[6]) {
                std::cout << "entering state 2(stop245,slow135):" << ",count=" << stateTransitionCount << std::endl;
                state = 2;
                stateTransitionCount++; 
                legSet[2] = legSet[4] = legSet[6] = false;
            }
            break;
        case 2:
            this->joint2->SetVelocity(0, 0);  
            this->joint4->SetVelocity(0, 0);  
            this->joint6->SetVelocity(0, 0); 
            angle =  abs(this->joint5->GetAngle(0).Degree());
            if ((abs(this->joint1->GetAngle(0).Degree()) % 360) == angle_switch_speed) {
                this->joint1->SetVelocity(0, 0);  
                legSet[1] = true;
            } else {
                this->joint1->SetVelocity(0, vel_slow);
            }
            if ((abs(this->joint3->GetAngle(0).Degree()) % 360) == angle_switch_speed) {
                this->joint3->SetVelocity(0, 0);  
                legSet[3] = true;
            } else {
                this->joint3->SetVelocity(0, vel_slow);
            }

            if ((abs(this->joint5->GetAngle(0).Degree()) % 360) == angle_switch_speed) {
                this->joint5->SetVelocity(0, 0);  
                legSet[5] = true;
            } else {
                this->joint5->SetVelocity(0, vel_slow);
            }

            if (legSet[1] && legSet[3] && legSet[5]) {
                std::cout << "entering state 3(stop246,fast135):"<< angle << ",count=" << stateTransitionCount <<  std::endl;
                state = 3;
                stateTransitionCount++; 
                legSet[1] = legSet[3] = legSet[5] = false;
            }
            break;
        case 3:
            this->joint2->SetVelocity(0, 0);  
            this->joint4->SetVelocity(0, 0);  
            this->joint6->SetVelocity(0, 0); 
            angle =  abs(this->joint5->GetAngle(0).Degree());
            if ((abs(this->joint1->GetAngle(0).Degree()) % 360) == angle_stand) {
                this->joint1->SetVelocity(0, 0);  
                legSet[1] = true;
            } else {
                this->joint1->SetVelocity(0, vel_fast);
            }

            if ((abs(this->joint3->GetAngle(0).Degree()) % 360) == angle_stand) {
                this->joint3->SetVelocity(0, 0);  
                legSet[3] = true;
            } else {
                this->joint3->SetVelocity(0, vel_fast);
            }

            if ((abs(this->joint5->GetAngle(0).Degree()) % 360) == angle_stand) {
                this->joint5->SetVelocity(0, 0);  
                legSet[5] = true;
            } else {
                this->joint5->SetVelocity(0, vel_fast);
            }

            if (legSet[1] && legSet[3] && legSet[5]) {
                std::cout << "entering state 4(slow246,stop135):" << angle << ",count=" << stateTransitionCount << std::endl;
                state = 4;
                stateTransitionCount++; 
                legSet[1] = legSet[3] = legSet[5] = false;
            }
            break;
        case 4:
            this->joint1->SetVelocity(0, 0);
            this->joint3->SetVelocity(0, 0);
            this->joint5->SetVelocity(0, 0);
            angle =  abs(this->joint2->GetAngle(0).Degree());

            if ((abs(this->joint2->GetAngle(0).Degree()) % 360) == angle_switch_speed) {
                this->joint2->SetVelocity(0, 0);  
                legSet[2] = true;
            } else {
                this->joint2->SetVelocity(0, vel_slow);
            }
            if ((abs(this->joint4->GetAngle(0).Degree()) % 360) == angle_switch_speed) {
                this->joint4->SetVelocity(0, 0);  
                legSet[4] = true;
            } else {
                this->joint4->SetVelocity(0, vel_slow);
            }

            if ((abs(this->joint6->GetAngle(0).Degree()) % 360) == angle_switch_speed) {
                this->joint6->SetVelocity(0, 0);  
                legSet[6] = true;
            } else {
                this->joint6->SetVelocity(0, vel_slow);
            }

            if (legSet[2] && legSet[4] && legSet[6]) {
                std::cout << "entering state 1(fast246,stop135):" << angle << ",count=" << stateTransitionCount <<  std::endl;
                state = 1;
                stateTransitionCount++; 
                legSet[2] = legSet[4] = legSet[6] = false;
            }
            break;

    } /* end switch */
    return;
}

static bool legSet[7] = {false};
void ModelPush::drive135(double v, int thetaS)
{
    if ((abs(this->joint1->GetAngle(0).Degree()) % 360) == thetaS) {
        this->joint1->SetVelocity(0, 0);  
        legSet[1] = true;
    } else {
        this->joint1->SetVelocity(0, v);
    }

    if ((abs(this->joint3->GetAngle(0).Degree()) % 360) == thetaS) {
        this->joint3->SetVelocity(0, 0);  
        legSet[3] = true;
    } else {
        this->joint3->SetVelocity(0, v);
    }

    if ((abs(this->joint5->GetAngle(0).Degree()) % 360) == thetaS) {
        this->joint5->SetVelocity(0, 0);  
        legSet[5] = true;
    } else {
        this->joint5->SetVelocity(0, v);
    }
    return;
}

void ModelPush::drive246(double v, int thetaS)
{
    if ((abs(this->joint2->GetAngle(0).Degree()) % 360) == thetaS) {
        this->joint2->SetVelocity(0, 0);  
        legSet[2] = true;
    } else {
        this->joint2->SetVelocity(0, v);
    }

    if ((abs(this->joint4->GetAngle(0).Degree()) % 360) == thetaS) {
        this->joint4->SetVelocity(0, 0);  
        legSet[4] = true;
    } else {
        this->joint4->SetVelocity(0, v);
    }

    if ((abs(this->joint6->GetAngle(0).Degree()) % 360) == thetaS) {
        this->joint6->SetVelocity(0, 0);  
        legSet[6] = true;
    } else {
        this->joint6->SetVelocity(0, v);
    }
    return;
}

void ModelPush::controller_angle2(void)
{
    static int state = 0, stateTransitionCount = 0;
    static common::Time t0;
    int bound = 5;
    int vel_fast = 1.5;//5;
    int vel_slow = .5;//2.5;
    int angle = 0;
    int thetaS = 20;

    switch (state) {
        case 0: // START state
            std::cout << "FSM begins" << std::endl;
            state = 1;
            std::cout << "entering state 1:" << ",count=" << stateTransitionCount <<  std::endl;
            break;
        case 1:
            drive135(vel_fast, thetaS);
            if (legSet[1] && legSet[3] && legSet[5]) {
                drive246(vel_slow, thetaS);
            } else {
                drive246(0, thetaS);
            }

            if (0&&legSet[2] && legSet[4] && legSet[6] && legSet[1] && legSet[3] && legSet[5]) {
                std::cout << "entering state 2(stop245,slow135):" << ",count=" << stateTransitionCount << std::endl;
                //                state = 2;
                stateTransitionCount++; 
                legSet[1] = legSet[3] = legSet[5] = legSet[2] = legSet[4] = legSet[6] = false;
            }
            break;
        case 2:
            drive135(vel_slow, thetaS);
            drive246(vel_slow, thetaS);

            if (legSet[2] && legSet[4] && legSet[6] && legSet[1] && legSet[3] && legSet[5]) {
                std::cout << "entering state 3(stop246,fast135):"<< angle << ",count=" << stateTransitionCount <<  std::endl;
                state = 3;
                stateTransitionCount++; 
                legSet[1] = legSet[3] = legSet[5] = legSet[2] = legSet[4] = legSet[6] = false;
            }
            break;
        case 3:
            this->joint2->SetVelocity(0, 0);  
            this->joint4->SetVelocity(0, 0);  
            this->joint6->SetVelocity(0, 0); 
            angle =  abs(this->joint5->GetAngle(0).Degree());
            if ((abs(this->joint1->GetAngle(0).Degree()) % 360) == 180) {
                this->joint1->SetVelocity(0, 0);  
                legSet[1] = true;
            } else {
                this->joint1->SetVelocity(0, vel_fast);
            }

            if ((abs(this->joint3->GetAngle(0).Degree()) % 360) == 180) {
                this->joint3->SetVelocity(0, 0);  
                legSet[3] = true;
            } else {
                this->joint3->SetVelocity(0, vel_fast);
            }

            if ((abs(this->joint5->GetAngle(0).Degree()) % 360) == 180) {
                this->joint5->SetVelocity(0, 0);  
                legSet[5] = true;
            } else {
                this->joint5->SetVelocity(0, vel_fast);
            }

            if (legSet[1] && legSet[3] && legSet[5]) {
                std::cout << "entering state 4(slow246,stop135):" << angle << ",count=" << stateTransitionCount << std::endl;
                state = 4;
                stateTransitionCount++; 
                legSet[1] = legSet[3] = legSet[5] = false;
            }
            break;
        case 4:
            this->joint1->SetVelocity(0, 0);
            this->joint3->SetVelocity(0, 0);
            this->joint5->SetVelocity(0, 0);
            angle =  abs(this->joint2->GetAngle(0).Degree());

            if ((abs(this->joint2->GetAngle(0).Degree()) % 360) == 359) {
                this->joint2->SetVelocity(0, 0);  
                legSet[2] = true;
            } else {
                this->joint2->SetVelocity(0, vel_slow);
            }
            if ((abs(this->joint4->GetAngle(0).Degree()) % 360) == 359) {
                this->joint4->SetVelocity(0, 0);  
                legSet[4] = true;
            } else {
                this->joint4->SetVelocity(0, vel_slow);
            }

            if ((abs(this->joint6->GetAngle(0).Degree()) % 360) == 359) {
                this->joint6->SetVelocity(0, 0);  
                legSet[6] = true;
            } else {
                this->joint6->SetVelocity(0, vel_slow);
            }

            if (legSet[2] && legSet[4] && legSet[6]) {
                std::cout << "entering state 1(fast246,stop135):" << angle << ",count=" << stateTransitionCount <<  std::endl;
                state = 1;
                stateTransitionCount++; 
                legSet[2] = legSet[4] = legSet[6] = false;
            }
            break;

    } /* end switch */
    return;
}

