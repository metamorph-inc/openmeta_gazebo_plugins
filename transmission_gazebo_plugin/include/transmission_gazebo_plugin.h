/*
*  Name: transmission_gazebo_plugin.h
*  Author: Joseph Coombe
*  Date: 06/23/2018
*  Description:
*   Transmission Gazebo Plugin
*/

#ifndef _TRANSMISSION_GAZEBO_PLUGIN_HH_
#define _TRANSMISSION_GAZEBO_PLUGIN_HH_

#include <thread>

// Gazebo dependencies
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{

  class TransmissionGazeboPlugin: public ModelPlugin
  {

    public:
      TransmissionGazeboPlugin();
      ~TransmissionGazeboPlugin();
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      void OnUpdate(const common::UpdateInfo & _info);

    private:
      void SetTargetPosition(const double & _pos);

      /// \brief Pointer to the model.
      physics::ModelPtr model_;

      /// \brief Pointer to joints
      physics::JointPtr joint_a_;
      physics::JointPtr joint_b_;

      double fwd_mech_reduction_;
      double bwd_mech_reduction_;

      double max_trans_torque_;

      /// \brief Pointer to the update connection event
      event::ConnectionPtr update_connection_;

  };

}


#endif /* _TRANSMISSION_GAZEBO_PLUGIN_HH_ */
