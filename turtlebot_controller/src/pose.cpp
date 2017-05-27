#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "pose"

class ROSnode
{
private:
	ros::NodeHandle Handle;
	double x, y, yaw, roll, pitch, t;
	ros::Publisher posePub;
	ros::Subscriber odomSub;
	
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	
public:
	bool Prepare();
	void runContinuously();
};

bool ROSnode::Prepare()
{
	roll = 0.0;
	pitch = 0.0;
	t = -1.0;
	
	/*if(!Handle.getParam(ros::this_node::getName()+"/x", x))
		return false;
	if(!Handle.getParam(ros::this_node::getName()+"/y", y))
		return false;
	if(!Handle.getParam(ros::this_node::getName()+"/yaw", yaw))
		return false;*/
	
	odomSub = Handle.subscribe("/odom", 10, &ROSnode::odomCallback, this);
	posePub = Handle.advertise<geometry_msgs::PoseStamped>("/pose", 10);
	
	return true;
}

void ROSnode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	if(t < 0) //initialize
	{
		t = msg->header.stamp.toSec();
		return;
	}
	
	//update parameters
	float v = msg->twist.twist.linear.x;
	float w = msg->twist.twist.angular.z;
	double dt = msg->header.stamp.toSec() - t;
	x = x + v*cos(yaw)*dt;
	y = y + v*sin(yaw)*dt;
	yaw = yaw + w*dt;
	t = msg->header.stamp.toSec();
	
	//publish pose
	geometry_msgs::PoseStamped out;
	out.header = msg->header;
	out.header.frame_id = "/base_link";
	//out.pose.orientation <- yawToQuaternion(yaw);
	tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
	out.pose.orientation.x = q.getX();
	out.pose.orientation.y = q.getY();
	out.pose.orientation.z = q.getZ();
	out.pose.orientation.w = q.getW();
	out.pose.position.x = x;
	out.pose.position.y = y;
	out.pose.position.z = 0.0;
	posePub.publish(out);
}

void ROSnode::runContinuously() {
	ROS_INFO("Node %s running continuously.", ros::this_node::getName().c_str());
	
	ros::spin();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, NAME_OF_THIS_NODE);
	
	ROSnode pNode;
	if(!pNode.Prepare())
	{
		puts("Failed to load node");
		return 0;
	}
	pNode.runContinuously();
}

	