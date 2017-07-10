#include "ros/ros.h"
#include "polygons/DrawAction.h"
#include "actionlib/server/simple_action_server.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "math.h"
#include "boost/math/constants/constants.hpp"
#include "ros/time.h"

class ROSnode
{
private:
	ros::NodeHandle Handle;
	actionlib::SimpleActionServer<polygons::DrawAction> server;
	ros::Publisher movePub;
	ros::Subscriber poseSub;
	turtlesim::Pose actualPose;
	polygons::DrawFeedback feedBack;
	polygons::DrawResult result;
	
	void poseCallBack(const turtlesim::Pose::ConstPtr& msg);
	
public:
	ROSnode() : server(Handle, "draw", boost::bind(boost::bind(&ROSnode::execute, this, _1), _1, &server), false)
	{
		ROS_INFO("Starting server");
		server.start();
	}
	void Prepare();
	void runContinuously();
	void execute(const polygons::DrawGoalConstPtr& goal);
};

void ROSnode::Prepare()
{
	poseSub = Handle.subscribe("turtle1/pose", 1, &ROSnode::poseCallBack, this);
	movePub = Handle.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
}

void ROSnode::poseCallBack(const turtlesim::Pose::ConstPtr& msg)
{
	actualPose.x = msg->x;
	actualPose.y = msg->y;
	actualPose.theta = msg->theta;
}


void ROSnode::execute(const polygons::DrawGoalConstPtr& goal)
{
	ROS_INFO("Goal received, started execution");
	geometry_msgs::Twist out;
	float alpha = 2 * boost::math::constants::pi<float>()/goal->sides; //external angle 2pi/n
	float angularVel = 0.25;
	float linearVel = 0.25;
	ros::Time t;
	for(int i = 0; i < goal->sides; i++)
	{
		t = ros::Time::now();
		while((ros::Time::now().toSec() - t.toSec()) < alpha/angularVel)
		{
			out.angular.z = angularVel;
			movePub.publish(out);
		}
		out.angular.z = 0;
		movePub.publish(out);
		t = ros::Time::now();
		while((ros::Time::now().toSec() - t.toSec()) < goal->length/linearVel)
		{
			out.linear.x = linearVel;
			movePub.publish(out);
		}
		out.linear.x = 0;
		movePub.publish(out);
		
		feedBack.drawn_sides = i + 1;
		server.publishFeedback(feedBack);
	}
	result.success = true;
	server.setSucceeded(result);
	ROS_INFO("Goal achieved");
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