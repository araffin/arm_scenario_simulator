#include <boost/bind.hpp>
#include <stdio.h>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include "geometry_msgs/Vector3.h"

namespace gazebo
{
  class MeshBox : public ModelPlugin
  {
    private:
      // Pointer to the model
      physics::ModelPtr model;
      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;
      math::Vector3 size;
      ros::NodeHandle rosNode;
      ros::Publisher rosPub;

    public:
      MeshBox(): rosNode("mesh_bounding_box")
      {
      }

      bool getBoundingBox()
      {

      }

      void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
      {
        // Store the pointer to the model
        this->model = _parent;
        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized())
        {
          ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
          return;
        }

        this->rosPub = this->rosNode.advertise<geometry_msgs::Vector3>(this->model->GetName(), 1);

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MeshBox::OnUpdate, this, _1));
      }

      void OnUpdate(const common::UpdateInfo & /*_info*/)
      {
        updateBoundingBox();
      }

      /**
       * Get a rough estimate of the bounding box for a given model
       * FIXME: take the pose of each collision box into account
       * (position + orientation)
       */
      void updateBoundingBox()
      {
        math::Vector3 size(0, 0, 0);
        // This method does seems to return the last bounding box
        // math::Vector3 size2 = this->model->GetBoundingBox().GetSize();

        for (unsigned int i = 0; i < this->model->GetChildCount(); i ++)
        {
          gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(this->model->GetChild(i));
          if(body)
          {
            for (unsigned int j = 0; j < body->GetChildCount(); j++)
            {
              gazebo::physics::CollisionPtr geom = boost::dynamic_pointer_cast<gazebo::physics::Collision>(body->GetChild(j));

              if(geom)
              {
                math::Vector3 tmp_size = geom->GetBoundingBox().GetSize();
                // std::cout << geom->GetBoundingBox().GetCenter() << '\n';
                // std::cout << geom->GetName() << " size: (" << tmp_size.x << "," << tmp_size.y << "," << tmp_size.z << ")" << '\n';
                size.x = tmp_size.x > size.x ? tmp_size.x : size.x;
                size.y = tmp_size.y > size.y ? tmp_size.y : size.y;
                size.z = tmp_size.z > size.z ? tmp_size.z : size.z;
              }
            }
          }
        }
        // std::cout << "Final size: (" << size.x << "," << size.y << "," << size.z << ")" << '\n';
        // std::cout << "Final size 2: (" << size2.x << "," << size2.y << "," << size2.z << ")" << '\n';
        // Create and publish ros message
        geometry_msgs::Vector3 msg;
        msg.x = size.x;
        msg.y = size.y;
        msg.z = size.z;
        this->rosPub.publish(msg);
      }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MeshBox)
}
