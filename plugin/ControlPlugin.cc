#include "ControlPlugin.hh"
#define DISTANCE_THRESHOLD 0.4
#define RAY_THRESHOLD 0.9
#define P_GAIN 195.5
#define I_GAIN 177.5
#define D_GAIN 1.95
#define TARGET_POSITION 0.3

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
	for(int i = 0; i < this->laserPtr-> RangeCount(); i++){

		if( this->laserPtr->Range(i) < DISTANCE_THRESHOLD*this->laserPtr->RangeMax()){
			count += 1;
		}

	}
	this->laserPtr->SetActive(true);

	if(count >= RAY_THRESHOLD*samples){
		
		SetPIDJoint();	
	}
}

void ControlPlugin::InitPIDJoint(){

	this->pid_right = common::PID(P_GAIN, I_GAIN, D_GAIN);
	this->pid_left = common::PID(P_GAIN, I_GAIN, D_GAIN);
	this->pid_back_right = common::PID(P_GAIN, I_GAIN, D_GAIN);
	this->pid_back_left = common::PID(P_GAIN, I_GAIN, D_GAIN);

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
	
	this->model->GetJointController()->SetPositionTarget(this->rightJoint->GetScopedName(), TARGET_POSITION);	 
	this->model->GetJointController()->SetPositionTarget(this->leftJoint->GetScopedName(), TARGET_POSITION); 	
	this->model->GetJointController()->SetPositionTarget(this->backRightJoint->GetScopedName(), TARGET_POSITION);
	this->model->GetJointController()->SetPositionTarget(this->backLeftJoint->GetScopedName(), TARGET_POSITION);

}