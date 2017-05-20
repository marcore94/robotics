#include "ControlFork.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ControlFork)

void ControlFork::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

	GZ_ASSERT(_parent, "ControlFork _parent pointer is NULL");
	GZ_ASSERT(_sdf, "ControlFork _sdf pointer is NULL");
	
	if (_parent == NULL || _sdf == NULL)
	{
		gzerr << "Failed to load FollowerPlugin. NULL model or sdf" << std::endl;
		return;
	}
	
	this->model = _parent;
	this->sdf = _sdf;
	this->forkPtr = this->model->GetLink("elevating_fork");

	if(!forkPtr)
		std::cout << "ERROR: forkPtr is NULL\n";

	sensors::SensorPtr sensor = sensors::get_sensor("laser");
	if (sensor->Type() == "ray")
	  	{
			sensors::RaySensorPtr raySensor =
				std::dynamic_pointer_cast<sensors::RaySensor>(sensor);
			if (raySensor)
			{
				this->laserPtr = raySensor;
				this->samples = this->laserPtr->RayCount();
				}
		}

	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
		boost::bind(&ControlFork::OnUpdate, this, _1));
}

void ControlFork::OnUpdate(const common::UpdateInfo & /*_info*/) {

	count = 0;
	this->laserPtr->SetActive(false);
	for(int i =0; i < this->laserPtr-> RangeCount(); i++){

		if( this->laserPtr->Range(i) < 0.4*this->laserPtr->RangeMax()){
			count += 1;
		}

	}
	this->laserPtr->SetActive(true);

	if(count>=0.9*samples){
		
		PIDJoint pid;
		pid.Load(this->model, this->sdf);			
		//this->forkPtr->SetForce(math::Vector3(0, 0, 1300));
	}
}