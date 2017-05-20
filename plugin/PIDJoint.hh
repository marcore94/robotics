#ifndef _PID_JOINT_HH_
#define _PID_JOINT_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo{
	class GAZEBO_VISIBLE PIDJoint : public ModelPlugin {
		public: PIDJoint();
		public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
		private: physics::ModelPtr model;
		private: sdf::ElementPtr sdf;
		private: physics::JointPtr backRightJoint;
		private: physics::JointPtr backLeftJoint;
		private: physics::JointPtr leftJoint;
		private: physics::JointPtr rightJoint;
		private: common::PID pid_right;
		private: common::PID pid_left;
		private: common::PID pid_back_right;
		private: common::PID pid_back_left;
	};
}
#endif