#include <ros/ros.h>
#include <ros/time.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/math/Rand.hh>
#include <gazebo/sensors/GpsSensor.hh>
#include <sensor_msgs/NavSatFix.h>
#include <stdio.h>
#include <sensor_msgs/NavSatFix.h>


#define NAME_OF_THIS_NODE "gps_plugin"
#define PROB_SIGNAL_LOST 0.1
#define PROB_TRANSLATION 0.05
#define MIN_FAULT_TIME 5
#define MAX_FAULT_TIME 10
#define DELTA_TRANSLATION 0.01

namespace gazebo
{

class GPSRealPlugin : public SensorPlugin {
	public: GPSRealPlugin();
	public: virtual ~GPSRealPlugin();
	public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
	protected: virtual void OnUpdate(sensors::GpsSensorPtr _sensor);
	protected: virtual void OnWorldUpdate(const common::UpdateInfo &_info);
	protected: sensors::GpsSensorPtr parentSensor;
	private: event::ConnectionPtr connection;
	private: event::ConnectionPtr updateConnection;
	private: ros::NodeHandle Handle;
	private: ros::Publisher pub;
	private: int translated;
	private: ros::Time startingFault;
	private: double faultDuration;
};

// Constructor
GPSRealPlugin::GPSRealPlugin()
{
}

// Destructor
GPSRealPlugin::~GPSRealPlugin()
{
	this->parentSensor->DisconnectUpdated(this->connection);
	this->parentSensor.reset();
}

// Load the controller
void GPSRealPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
	int argc = 0;
	char **argv = NULL;
	ros::init(argc, argv, NAME_OF_THIS_NODE);
	this->pub = Handle.advertise<sensor_msgs::NavSatFix>("/fix", 10);
	
	// Make sure the ROS node for Gazebo has already been initalized
	if (!ros::isInitialized())
	{
		ROS_FATAL_STREAM_NAMED("template", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
			<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
		return;
	}
	
	this->parentSensor = std::dynamic_pointer_cast<sensors::GpsSensor>(_parent);
	if (!this->parentSensor)
		gzthrow("FaultyGPSPlugin requires a gps sensor as its parent.");
	this->parentSensor->SetActive(true);
	this->parentSensor->SetUpdateRate(1);
	translated = 0;
	
	this->connection = this->parentSensor->ConnectUpdated(std::bind(&GPSRealPlugin::OnUpdate, this, this->parentSensor));
	
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GPSRealPlugin::OnWorldUpdate, this, _1));
	
	ROS_INFO("GPS loaded correctly");
}

void GPSRealPlugin::OnUpdate(sensors::GpsSensorPtr _sensor)
{
	if(math::Rand::GetDblUniform() < PROB_SIGNAL_LOST && translated == 0)
	{
		_sensor->SetActive(false);
		startingFault = ros::Time::now();
		faultDuration = math::Rand::GetDblUniform(MIN_FAULT_TIME, MAX_FAULT_TIME);
		ROS_INFO("GPS signal lost");
	}
	if(_sensor->IsActive() && math::Rand::GetDblUniform() < PROB_TRANSLATION && translated == 0)
	{
		translated = 1;
		startingFault = ros::Time::now();
		faultDuration = math::Rand::GetDblUniform(MIN_FAULT_TIME, MAX_FAULT_TIME);
		ROS_INFO("Translation enabled");
	}
	if(translated == 1 && (ros::Time::now().toSec() - startingFault.toSec()) > faultDuration)
	{
		translated = 0;
		ROS_INFO("Translation disabled");
	}
	sensor_msgs::NavSatFix out;
	out.altitude = _sensor->Altitude();
	out.latitude = _sensor->Latitude().Radian() * 180 / 3.14159265 + DELTA_TRANSLATION * translated;
	out.longitude = _sensor->Longitude().Radian() * 180 / 3.14159265 + DELTA_TRANSLATION * translated;
	pub.publish(out);
	/*if(translated == 1)
	{
		double lat = _sensor->Latitude().Radian() * 180 / 3.14159265;
		ROS_INFO("Non translated lat = %f\nTranslated lat = %f", lat, out.latitude);
	}*/
}

void GPSRealPlugin::OnWorldUpdate(const common::UpdateInfo& _info)
{
	if(!this->parentSensor->IsActive())
	{
		if((ros::Time::now().toSec() - startingFault.toSec()) > faultDuration)
		{
			this->parentSensor->SetActive(true);
			translated = 0;
			ROS_INFO("GPS signal ok");
		}
	}
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GPSRealPlugin);
}

