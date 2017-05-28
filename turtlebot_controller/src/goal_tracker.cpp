#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "turtlebot_controller/GetGoal.h"

#define NAME_OF_THIS_NODE "goal_tracker"

class ROSnode
{
private:
	ros::NodeHandle Handle;
	ros::Subscriber goalSub;
	ros::ServiceServer goalSr;
	bool gotGoal;
	geometry_msgs::Pose goalPose;
	
	void planCallback(const geometry_msgs::Pose::ConstPtr& msg);
	bool goalService(turtlebot_controller::GetGoal::Request &req, turtlebot_controller::GetGoal::Response &res); 
	
public:
	void Prepare();
	void runContinuously();
};

void ROSnode::Prepare()
{
	gotGoal = false;
	
	goalSub = Handle.subscribe("/getGoal", 10, &ROSnode::planCallback, this);
	goalSr = Handle.advertiseService("goal", &ROSnode::goalService, this);
}


bool ROSnode::goalService(turtlebot_controller::GetGoal::Request& req, turtlebot_controller::GetGoal::Response& res)
{
	if(!gotGoal)
		return false;
	if(req.go)
		res.goal = goalPose;
	return true;
}

void ROSnode::planCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	goalPose.position = msg->position;
	goalPose.orientation = msg->orientation;
	gotGoal = true;
}

void ROSnode::runContinuously()
{
	ROS_INFO("Node %s running continuously.", ros::this_node::getName().c_str());
	
	ros::spin();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, NAME_OF_THIS_NODE);
	
	ROSnode sNode;
	sNode.Prepare();
	sNode.runContinuously();
}
