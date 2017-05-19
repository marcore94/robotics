#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
namespace gazebo {
	class ControlFork : public ModelPlugin {
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

			GZ_ASSERT(_parent, "ControlFork _parent pointer is NULL");
			GZ_ASSERT(_sdf, "ControlFork _sdf pointer is NULL");
			
			if (_parent == NULL || _sdf == NULL)
			{
				gzerr << "Failed to load FollowerPlugin. NULL model or sdf" << std::endl;
				return;
			}
			
			this->model = _parent;
			
			if(_sdf->HasElement("elevating_fork")){			
				this->forkPtr = this->model->GetLink("elevating_fork");
			} 

			sensors::SensorPtr sensor = sensors::get_sensor("laser");
			if (sensor->Type() == "ray")
			  	{
					sensors::RaySensorPtr raySensor =
						std::dynamic_pointer_cast<sensors::RaySensor>(sensor);
					if (raySensor)
					{
					this->laserPtr = raySensor;
					}
				}

			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&ControlFork::OnUpdate, this, _1));
		}

		public: void OnUpdate(const common::UpdateInfo & /*_info*/) {

			this->laserPtr->SetActive(false);
			for(int i =0; i < this->laserPtr-> RangeCount(); i++){

				if( this->laserPtr->Range(i) < this->laserPtr->RangeMax()){
					std::cout << "Range minore di max per raggio " << i << " \n";
					object = 1;
				}

			}
			this->laserPtr->SetActive(true);

			if(object == 1){
				std::cout << "alzo\n";
				this->model->SetLinearVel(math::Vector3(1.30, 0, 0));
				object=0;
				//this->forkPtr->SetForce(math::Vector3(0, 0, 13));
			}
		}

		private: physics::ModelPtr model;
		private: event::ConnectionPtr updateConnection;
		private: physics::LinkPtr forkPtr;						//puntatore alle forche
		private: sensors::RaySensorPtr laserPtr;				//puntatore al laser
		private: bool object = 0;
	};

	GZ_REGISTER_MODEL_PLUGIN(ControlFork);
}