#include "ControlPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)

void ControlPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

	GZ_ASSERT(_parent, "ControlPlugin _parent pointer is NULL");
	GZ_ASSERT(_sdf, "ControlPlugin _sdf pointer is NULL");
	
	if (_parent == NULL || _sdf == NULL)
	{
		gzerr << "Failed to load ControlPlugin. NULL model or sdf" << std::endl;
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

	InitPIDJoint();

	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
		boost::bind(&ControlPlugin::OnUpdate, this, _1));
}

void ControlPlugin::OnUpdate(const common::UpdateInfo & /*_info*/) {

	count = 0;
	this->laserPtr->SetActive(false);
	for(int i =0; i < this->laserPtr-> RangeCount(); i++){

		if( this->laserPtr->Range(i) < 0.4*this->laserPtr->RangeMax()){
			count += 1;
		}

	}
	this->laserPtr->SetActive(true);

	if(count>=0.9*samples){
		
		SetPIDJoint();	
	}
}

void ControlPlugin::InitPIDJoint(){

	this->pid_right = common::PID(800, 500, 0);//800, x, 0
	this->pid_left = common::PID(800, 500, 0);
	this->pid_back_right = common::PID(800, 500, 0);
	this->pid_back_left = common::PID(800, 500, 0);
}

void ControlPlugin::SetPIDJoint(){

	this->backRightJoint = this->model->GetJoint("right_back_prism");
	this->backLeftJoint = this->model->GetJoint("left_back_prism");
	this->leftJoint = this->model->GetJoint("left_prism");
	this->rightJoint = this->model->GetJoint("right_prism");

	this->model->GetJointController()->SetPositionPID(this->rightJoint->GetScopedName(), this->pid_right);
	this->model->GetJointController()->SetPositionPID(this->leftJoint->GetScopedName(), this->pid_left);
	this->model->GetJointController()->SetPositionPID(this->backRightJoint->GetScopedName(), this->pid_back_right);
	this->model->GetJointController()->SetPositionPID(this->backLeftJoint->GetScopedName(), this->pid_back_left);
	
	this->model->GetJointController()->SetPositionTarget(this->rightJoint->GetScopedName(), 0.2);	//troppo basso : 0.135, 
	this->model->GetJointController()->SetPositionTarget(this->leftJoint->GetScopedName(), 0.2); 	//troppo alto : 0.2
	this->model->GetJointController()->SetPositionTarget(this->backRightJoint->GetScopedName(), 0.2);
	this->model->GetJointController()->SetPositionTarget(this->backLeftJoint->GetScopedName(), 0.2);
}