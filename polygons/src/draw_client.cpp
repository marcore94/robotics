#include "ros/ros.h"
#include "polygons/Draw.h"
#include "polygons/DrawAction.h"
#include "actionlib/client/simple_action_client.h"

typedef actionlib::SimpleActionClient<polygons::DrawAction> Client;

class ROSnode
{
private:
	Client client;
	ros::NodeHandle Handle;
	ros::Subscriber drawSub;
	
	void drawCallback(const polygons::Draw::ConstPtr& msg);
	
public:
	ROSnode() : client("draw", true)
	{
		ROS_INFO("Waiting for action server to start.");
		client.waitForServer();
		ROS_INFO("Action server started, sending goal.");
	}
	
	void Prepare();
	void doneCb(const actionlib::SimpleClientGoalState& state, 
				const polygons::DrawResultConstPtr& result);
	void feedbackCb(const polygons::DrawFeedbackConstPtr& feedback);
	void runContinuously();
};

void ROSnode::Prepare()
{	
	drawSub = Handle.subscribe("/draw", 1, &ROSnode::drawCallback, this);
}

void ROSnode::drawCallback(const polygons::Draw::ConstPtr& msg)
{
	ROS_INFO("Received polygon request");
	if(msg->length == 0)
	{
		ROS_ERROR("Cannot draw polygon with zero length");
		return;
	}
	if(msg->sides < 3)
	{
		ROS_ERROR("Cannot draw polygon with less than 3 sides");
		return;
	}
	polygons::DrawGoal goal;
	goal.length = msg->length;
	goal.sides = msg->sides;
	client.sendGoal(goal,
					boost::bind(&ROSnode::doneCb, this, _1, _2),
					Client::SimpleActiveCallback(),
					boost::bind(&ROSnode::feedbackCb, this, _1));
	ROS_INFO("Request sent to server");
}

void ROSnode::doneCb(const actionlib::SimpleClientGoalState& state, const polygons::DrawResultConstPtr& result)
{
	if(result->success)
		ROS_INFO("Draw completed successfully");
	else
		ROS_INFO("Draw not completed, wall hit");
}

void ROSnode::feedbackCb(const polygons::DrawFeedbackConstPtr& feedback)
{
	ROS_INFO("Side number %i drawn", feedback->drawn_sides);
}

void ROSnode::runContinuously() {
	ROS_INFO("Node %s running continuously.", ros::this_node::getName().c_str());
	
	ros::spin();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "draw_client");
	ROSnode draw_client;
	draw_client.Prepare();
	draw_client.runContinuously();
}
