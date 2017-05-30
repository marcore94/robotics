#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "turtlebot_controller/GetGoal.h"
#include "tf/transform_datatypes.h"
#include "math.h"

#define NAME_OF_THIS_NODE "control"

class ROSnode
{
private:
	ros::NodeHandle Handle;
	ros::Subscriber poseSub;
	ros::Publisher cmdPub;
	ros::ServiceClient goalClient;
	bool position, orientationCorrect, gotToGoal;
	geometry_msgs::Pose goal;
	
	void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	
public:
	void Prepare();
	void runContinuously();
};

void ROSnode::Prepare()
{
	position = false;
	orientationCorrect = false;
	gotToGoal = false;
	
	poseSub = Handle.subscribe("/pose", 10, &ROSnode::poseCallback, this);
	cmdPub = Handle.advertise<geometry_msgs::Twist>("cmd_auto", 10);
	
	goalClient = Handle.serviceClient<turtlebot_controller::GetGoal>("goal");
}

void ROSnode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::Twist out;
	double totalDistance;
	
	if(!gotToGoal)
	{
		turtlebot_controller::GetGoal srv;
		srv.request.go = true;
		if(goalClient.call(srv))
		{
			goal = srv.response.goal;
			gotToGoal = true;
			position = false;
			orientationCorrect = false;
			totalDistance = sqrt(pow(goal.position.x - msg->pose.position.x, 2) + 
				pow(goal.position.y - msg->pose.position.y, 2));
		}
		else 
			return;
	}
	
	if(!orientationCorrect)
	{
		double gx = goal.position.x;
		double gy = goal.position.y;
		double th = atan2(gy - msg->pose.position.y, gx - msg->pose.position.x);
		//yaw <- yawFromQuaternion(msg->pose.orientation)
		tf::Quaternion q;
		q.setX(msg->pose.orientation.x);
		q.setY(msg->pose.orientation.y);
		q.setZ(msg->pose.orientation.z);
		q.setW(msg->pose.orientation.w);
		double yaw = tf::getYaw(q);
		if(fabs(yaw - th) < 0.005)
			orientationCorrect = true;
		else
			out.angular.z = 0.03 * (yaw - th);
	}
	
	if(!position && orientationCorrect)
	{
		double actualDistance = sqrt(pow(goal.position.x - msg->pose.position.x, 2) + 
				pow(goal.position.y - msg->pose.position.y, 2));
		if(actualDistance < 0.1*0.1)
		{
			out.linear.x = 0.0;
			position = true;
		}
		else if(actualDistance <= totalDistance * 0.5 && totalDistance > 0.05)
		{
			totalDistance = totalDistance * 0.5;
			out.linear.x = 0.0;
			orientationCorrect = false;
		}
		else
		{
			if(actualDistance <= 10)
				out.linear.x = actualDistance * 0.01;
			else
				out.linear.x = 1;
		}
	}
	
	if(position && orientationCorrect)
		gotToGoal = false;
	
	cmdPub.publish(out);
}

void ROSnode::runContinuously() {
	ROS_INFO("Node %s running continuously.", ros::this_node::getName().c_str());
	
	ros::spin();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, NAME_OF_THIS_NODE);
	
	ROSnode cNode;
	cNode.Prepare();
	cNode.runContinuously();
}
