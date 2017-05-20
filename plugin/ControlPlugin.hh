#ifndef _CONTROL_PLUGIN_HH_
#define _CONTROL_PLUGIN_HH_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>

namespace gazebo{
	class GAZEBO_VISIBLE ControlPlugin : public ModelPlugin {
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
		public: void OnUpdate(const common::UpdateInfo & /*_info*/);
		public: void InitPIDJoint();
		public: void SetPIDJoint();
		private: physics::JointPtr backRightJoint;
		private: physics::JointPtr backLeftJoint;
		private: physics::JointPtr leftJoint;
		private: physics::JointPtr rightJoint;
		private: common::PID pid_right;
		private: common::PID pid_left;
		private: common::PID pid_back_right;
		private: common::PID pid_back_left;
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