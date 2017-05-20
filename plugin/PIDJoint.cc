#include "PIDJoint.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(PIDJoint)

PIDJoint::PIDJoint(){
	this->pid_right = common::PID(0.1, 0.1, 0.1);
	this->pid_left = common::PID(0.1, 0.1, 0.1);
	this->pid_back_right = common::PID(0.1, 0.1, 0.1);
	this->pid_back_left = common::PID(0.1, 0.1, 0.1);
}

void PIDJoint::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
	
	this->model = _model;
	this->sdf = _sdf;

	this->backRightJoint = this->model->GetJoint("right_back_prism");
	this->backLeftJoint = this->model->GetJoint("left_back_prism");
	this->leftJoint = this->model->GetJoint("left_prism");
	this->rightJoint = this->model->GetJoint("right_prism");

	this->model->GetJointController()->SetPositionPID(this->rightJoint->GetScopedName(), this->pid_right);
	this->model->GetJointController()->SetPositionPID(this->leftJoint->GetScopedName(), this->pid_left);
	this->model->GetJointController()->SetPositionPID(this->backRightJoint->GetScopedName(), this->pid_back_right);
	this->model->GetJointController()->SetPositionPID(this->backLeftJoint->GetScopedName(), this->pid_back_left);

	this->model->GetJointController()->SetPositionTarget(this->rightJoint->GetScopedName(), 0.05);
	this->model->GetJointController()->SetPositionTarget(this->leftJoint->GetScopedName(), 0.05);
	this->model->GetJointController()->SetPositionTarget(this->backRightJoint->GetScopedName(), 0.05);
	this->model->GetJointController()->SetPositionTarget(this->backLeftJoint->GetScopedName(), 0.05);
}