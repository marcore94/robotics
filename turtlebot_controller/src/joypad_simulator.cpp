#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "termios.h"
#include "stdio.h"
#include "signal.h"

#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "joypad_simulator"
#define KEYCODE_W 0x41
#define KEYCODE_S 0x42
#define KEYCODE_D 0x43 
#define KEYCODE_A 0x44

class ROSnode 
{
private:
	ros::NodeHandle Handle;
	ros::Publisher joyPub;
	
	float linearVel, angularVel;
	
public:
	void Prepare();
	void keyRun();
};

void ROSnode::Prepare()
{
	joyPub = Handle.advertise<sensor_msgs::Joy>("joy",1);
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
		ROS_DEBUG("value: 0x%02X\n", c);
		
		switch(c)
		{
			case KEYCODE_W:
				ROS_DEBUG("UP");
				linearVel = 1.0;
				dirty = true;
				break;
			case KEYCODE_S:
				ROS_DEBUG("DOWN");
				linearVel = -1.0;
				dirty = true;
				break;
			case KEYCODE_A:
				ROS_DEBUG("LEFT");
				angularVel = 1.0;
				dirty = true;
				break;
			case KEYCODE_D:
				ROS_DEBUG("RIGHT");
				angularVel = -1.0;
				dirty = true;
				break;
		}
		sensor_msgs::Joy msg;
		msg.axes.resize(3);
		msg.axes[1] = linearVel;
		msg.axes[2] = angularVel;
		if(dirty == true)
		{
			joyPub.publish(msg);
			dirty = false;
		}
	}
	return;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, NAME_OF_THIS_NODE);
	ROSnode joySim;
	joySim.Prepare();
	signal(SIGINT,quit);
	joySim.keyRun();
	
	return (0);
}
