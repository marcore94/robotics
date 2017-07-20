#include "ros/ros.h"
#include "termios.h"
#include "stdio.h"
#include "signal.h"
#include "geometry_msgs/Twist.h"

#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "keycontrol"
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_RIGHT 0x43 
#define KEYCODE_LEFT 0x44

class ROSnode 
{
private:
	ros::NodeHandle Handle;
	ros::Publisher keyPub;
	
	float linearVel, angularVel;
	
public:
	void Prepare();
	void keyRun();
};

void ROSnode::Prepare()
{
	keyPub = Handle.advertise<geometry_msgs::Twist>("cmd_vel",1);
	linearVel = 0;
	angularVel = 0;
	
	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
}

void ROSnode::keyRun()
{
	char c;
	bool dirty = false;
	
	//console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	//new line and end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);
	
	puts("---------------------------");
	puts("Simulating joypad");
	puts("---------------------------");
	puts("Use arrow keys to move");
	
	for(;;)
	{
		if(read(kfd, &c, 1) < 0)
		{
			perror("read():");
			exit(-1);
		}

		linearVel = angularVel = 0;
		
		geometry_msgs::Twist msg;

		switch(c)
		{
			case KEYCODE_UP:
				linearVel = -1.0;
				msg.linear.x = linearVel;
				dirty = true;
				break;
			case KEYCODE_DOWN:
				linearVel = 1.0;
				msg.linear.x = linearVel;
				dirty = true;
				break;
			case KEYCODE_LEFT:
				angularVel = -0.1;
				msg.angular.z = angularVel;
				dirty = true;
				break;
			case KEYCODE_RIGHT:
				angularVel = 0.1;
				msg.angular.z = angularVel;
				dirty = true;
				break;
		}

		if(dirty == true)
		{
			keyPub.publish(msg);
			dirty = false;
			usleep(500000);
			msg.linear.x = 0;
			msg.angular.z = 0;
			keyPub.publish(msg);
		}
	}
	return;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, NAME_OF_THIS_NODE);
	ROSnode keyControl;
	keyControl.Prepare();
	signal(SIGINT,quit);
	keyControl.keyRun();
	
	return (0);
}