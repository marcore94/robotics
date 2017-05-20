#ifndef _CONTROL_FORK_HH_
#define _CONTROL_FORK_HH_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include "PIDJoint.hh"

namespace gazebo{
	class GAZEBO_VISIBLE ControlFork : public ModelPlugin {
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
		public: void OnUpdate(const common::UpdateInfo & /*_info*/);
		private: event::ConnectionPtr updateConnection;
		private: physics::ModelPtr model;
		private: physics::LinkPtr forkPtr;						
		private: sensors::RaySensorPtr laserPtr;
		private: sdf::ElementPtr sdf;				
		private: int count;
		private: int samples;
	};
}
#endif