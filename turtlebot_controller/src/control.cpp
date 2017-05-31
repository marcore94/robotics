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
	ros::Subscriber goalSub;
	ros::Publisher cmdPub;
	ros::ServiceClient goalClient;
	bool position, orientationCorrect, gotToGoal, finalOrientation, gotNewGoal;
	geometry_msgs::Pose goal;
	double totalDistance;
	
	void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void goalCallback(const geometry_msgs::Pose::ConstPtr& msg);
	
public:
	void Prepare();
	void runContinuously();
};

void ROSnode::Prepare()
{
	position = false;
	orientationCorrect = false;
	gotToGoal = true;
	finalOrientation = false;
	gotNewGoal = false;
	totalDistance = 0;
	
	poseSub = Handle.subscribe("/pose", 10, &ROSnode::poseCallback, this);
	goalSub = Handle.subscribe("/goal", 1, &ROSnode::goalCallback, this);
	cmdPub = Handle.advertise<geometry_msgs::Twist>("cmd_auto", 10);
	
	goalClient = Handle.serviceClient<turtlebot_controller::GetGoal>("goal");
}

void ROSnode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::Twist out;
	
	if(gotNewGoal)
	{
		gotNewGoal = false;
		gotToGoal = false;
		position = false;
		orientationCorrect = false;
		finalOrientation = false;
		std::cout << "gx = " << goal.position.x << ", px = " << msg->pose.position.x << "\n";
		std::cout << "gy = " << goal.position.y << ", py = " << msg->pose.position.y << "\n";
		totalDistance = sqrt(pow((goal.position.x - msg->pose.position.x), 2) + pow((goal.position.y - msg->pose.position.y), 2));
	}
	
	if(!gotNewGoal && gotToGoal)
		return;
	
	
	std::cout << "total: " << totalDistance << "\n";
	if(!orientationCorrect && !gotToGoal)
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
		//double yaw = atan2(2 * (msg->pose.orientation.w * msg->pose.orientation.z + msg->pose.orientation.x * msg->pose.orientation.y),
		//				   1 - 2 * (pow(msg->pose.orientation.y, 2) * pow(msg->pose.orientation.z, 2)));
		std::cout << "yaw: " << yaw << "\nth: " << th << "\n";
		if(fabs(yaw - th) < 0.005)
		{
			puts("orientation true");
			orientationCorrect = true;
		}
		else
			out.angular.z = 0.03 * (th - yaw) / fabs(th - yaw);
	}
	
	if(!position && orientationCorrect && !gotToGoal)
	{
		puts("set linear vel");
		std::cout << "gx = " << goal.position.x << ", px = " << msg->pose.position.x << "\n";
		std::cout << "gy = " << goal.position.y << ", py = " << msg->pose.position.y << "\n";
		double actualDistance = pow(goal.position.x - msg->pose.position.x, 2) + pow(goal.position.y - msg->pose.position.y, 2);
		std::cout << "actual: " << actualDistance << "\n";
		if(actualDistance < 0.1 * 0.1)
		{
			out.linear.x = 0.0;
			position = true;
		}
		else
		{
			if(actualDistance <= totalDistance * 0.5 && totalDistance > 0.2)
			{
				totalDistance = totalDistance * 0.5;
				out.linear.x = 0.0;
				orientationCorrect = false;
			}
			else
			{
				if(actualDistance <= 3)
					out.linear.x = actualDistance * 0.1;
				else
				{
					if(actualDistance <= 10)
						out.linear.x = actualDistance * 0.01;
					else
						out.linear.x = 0.3;
				}
			}
		}
	}
	
	if(position && orientationCorrect && !finalOrientation && !gotToGoal)
	{
		double finalYaw = tf::getYaw(goal.orientation);
		double actualYaw = tf::getYaw(msg->pose.orientation);
		if(fabs(finalYaw - actualYaw) < 0.005)
		{
			puts("final orientation true");
			finalOrientation = true;
		}
		else
			out.angular.z = 0.03 * (finalYaw - actualYaw) / fabs(finalYaw - actualYaw);
	}
	
	if(position && orientationCorrect && finalOrientation)
	{
		gotToGoal = true;
	}
	
	cmdPub.publish(out);
}

void ROSnode::goalCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	goal.position = msg->position;
	goal.orientation = msg->orientation;
	
	gotNewGoal = true;
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
