#include <boost/bind.hpp>
#include <stdio.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  class ButtonPusher : public ModelPlugin
  {
    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    private: double k; // The spring constant
    private: double z0;  // the equilibrium position (with no gravity)
    private: double z_eq;  // the equilibrium position (with gravity)


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      if (_sdf->HasElement("k"))
      {
        this->k = _sdf->Get<double>("k");
      }
      else
      {
        this->k = 5;
      }
      this->z_eq = 0.015;
      double g = 9.81;
      double m = this->model->GetLink("button")->GetInertial()->GetMass();
      this->z0 = this->z_eq + (m * g) / this->k;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ButtonPusher::OnUpdate, this, _1));
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      gazebo::math::Vector3 p = this->model->GetLink("button")->GetRelativePose().pos;
      double force = - this->k * (p.z - this->z0);

      // Apply a force of on the axis 0
      this->model->GetJoint("glider")->SetForce(0, force);
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ButtonPusher)
}
