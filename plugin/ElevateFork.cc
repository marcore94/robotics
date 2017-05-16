#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
#include <stdio.h>

namespace gazebo {
	class ElevateFork : public ModelPlugin {
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
			this->model = _parent;
			this->fork = this->model->GetLink("elevating_fork");
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&ElevateFork::OnUpdate, this, _1));
		}
		public: void OnUpdate(const common::UpdateInfo & /*_info*/) {

			//comportamento decente
			this->fork->SetForce(math::Vector3(0, 0, 13));
			if(a<500){
				printf("X = %f\n", this->fork->GetRelativePose().pos.x);
				printf("Y = %f\n", this->fork->GetRelativePose().pos.y);
				printf("Z = %f\n", this->fork->GetRelativePose().pos.z);
				a++;
			}

			//"130 si vola"
			//this->fork->SetForce(math::Vector3(0, 0, 130));
		}

		private: physics::ModelPtr model;
		private: event::ConnectionPtr updateConnection;
		private: physics::LinkPtr fork;
		private: int a = 0;

	};

	GZ_REGISTER_MODEL_PLUGIN(ElevateFork);
}