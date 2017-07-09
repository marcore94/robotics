#include "ros/ros.h"
#include "polygons/DrawAction.h"
#include "actionlib/server/simple_action_server.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"

typedef actionlib::SimpleActionServer<polygons::DrawAction> Server;

class ROSnode
{
private:
	Server server;
	ros::NodeHandle Handle;
	ros::Publisher movePub;
	ros::Subscriber poseSub;
	geometry_msgs::Pose actualPose;
	geometry_msgs::Pose startingPose;
	
	void poseCallBack(const geometry_msgs::Pose::ConstPtr& msg);
	void execute(const polygons::DrawGoalConstPtr& goal, Server* server);
	
public:
	ROSnode() : server(Handle, "draw", boost::bind(boost::bind(&ROSnode::execute, this, _1, _2), _1, &server), false)
	{
		server.start();
	}
	
	void Prepare();
	void runContinuously();
};

void ROSnode::Prepare()
{
	poseSub = Handle.subscribe("", 1, &ROSnode::poseCallBack, this);
	movePub = Handle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}

void ROSnode::poseCallBack(const geometry_msgs::Pose::ConstPtr& msg)
{
	actualPose.position = msg->position;
	actualPose.orientation = msg->orientation;
}


void ROSnode::execute(const polygons::DrawGoalConstPtr& goal, Server* server)
{
	startingPose = actualPose;
	geometry_msgs::Twist out;
	movePub.publish(out);
}

void ROSnode::runContinuously() {
	ROS_INFO("Node %s running continuously.", ros::this_node::getName().c_str());
	
	ros::spin();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "draw_server");
	ROSnode draw_server;
	draw_server.Prepare();
	draw_server.runContinuously();
}