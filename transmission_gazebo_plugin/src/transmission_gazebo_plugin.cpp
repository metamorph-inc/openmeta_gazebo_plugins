/*
*  Name: transmission_gazebo_plugin.h
*  Author: Joseph Coombe
*  Date: 06/23/2018
*  Description:
*   Transmission Gazebo Plugin
*/

// Plugin header file
#include <transmission_gazebo_plugin.h>

#include <stdio.h>
#include <boost/bind.hpp>
#include <thread>

// Gazebo dependencies
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// ROS dependencies
#include "ros/ros.h"

namespace gazebo
{

  /// \brief Constructor
  TransmissionGazeboPlugin::TransmissionGazeboPlugin() {}

  /// \brief Destructor
  TransmissionGazeboPlugin::~TransmissionGazeboPlugin() {}

  /// \brief The load function is called by Gazebo when the plugin is
  /// inserted into a simulation (e.g. via URDF/SDF definition file)
  /// \param[in] _model A pointer to the model that this plugin is attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
  void TransmissionGazeboPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    std::cerr << "Loading TransmissionGazeboPlugin\n";
    // Store the model pointer for convenience.
    this->model_ = _model;


    //// IMPORT PLUGIN PARAMETERS FROM SDF ////
    std::string joint_a_name = ""; // default to empty string
    if ( _sdf->HasElement("jointA") ) {
      joint_a_name = _sdf->Get<std::string>("jointA");
    } else {
      ROS_WARN_NAMED("TransmissionGazeboPlugin", "TransmissionGazeboPlugin missing <jointA>!!!");
    }
    this->joint_a_ = this->model_->GetJoint(joint_a_name);

    std::string joint_b_name = ""; // default to empty string
    if ( _sdf->HasElement("jointB") ) {
      joint_b_name = _sdf->Get<std::string>("jointB");
    } else {
      ROS_WARN_NAMED("TransmissionGazeboPlugin", "TransmissionGazeboPlugin missing <jointB>!!!");
    }
    this->joint_b_ = this->model_->GetJoint(joint_b_name);

    this->fwd_mech_reduction_ = 1.0;
    if ( _sdf->HasElement("forwardMechanicalReduction") ) {
      this->fwd_mech_reduction_ = _sdf->Get<double>("forwardMechanicalReduction");
    } else {
      ROS_WARN_NAMED("TransmissionGazeboPlugin", "TransmissionGazeboPlugin missing <forwardMechanicalReduction>, defaults to \
      value \"%f\"", this->fwd_mech_reduction_);
    }

    this->bwd_mech_reduction_ = 1.0;
    if ( _sdf->HasElement("backwardMechanicalReduction") ) {
      this->bwd_mech_reduction_ = _sdf->Get<double>("backwardMechanicalReduction");
    } else {
      ROS_WARN_NAMED("TransmissionGazeboPlugin", "TransmissionGazeboPlugin missing <backwardMechanicalReduction>, defaults to \
      value \"%f\"", this->bwd_mech_reduction_);
    }

    // Get the maxTorque from SDF.
    this->max_trans_torque_ = 10.0;
    if ( _sdf->HasElement("maxTransmissionTorque") ) {
      this->max_trans_torque_ = _sdf->Get<double>("maxTransmissionTorque");
    } else {
      ROS_WARN_NAMED("TransmissionGazeboPlugin", "TransmissionGazeboPlugin missing <maxTransmissionTorque>, defaults to \
      value \"%f\" (N-m)", this->max_trans_torque_);
    }

    std::cerr << "Done Loading TransmissionGazeboPlugin" << '\n';

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin (
      boost::bind ( &TransmissionGazeboPlugin::OnUpdate, this, _1 ) );
  }

  /// \brief Function called at each simulation interation
  void TransmissionGazeboPlugin::OnUpdate(const common::UpdateInfo & _info)
  {
    this->joint_a_->SetProvideFeedback(true);
    double effort_a_ = this->joint_a_->GetForce(0);

    this->joint_b_->SetProvideFeedback(true);
    double effort_b_ = this->joint_b_->GetForce(0);

    this->joint_b_->SetForce(0, effort_a_);
    //this->joint_a_->SetForce(0, effort_b_);
  }

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(TransmissionGazeboPlugin);
}
