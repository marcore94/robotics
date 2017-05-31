#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "termios.h"
#include "stdio.h"
#include "signal.h"

#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "joypad_simulator"
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_RIGHT 0x43 
#define KEYCODE_LEFT 0x44

class ROSnode 
{
private:
	ros::NodeHandle Handle;
	ros::Publisher joyPub;
	
	int manMode, autMode;
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
	manMode = 0;
	autMode = 0;
	
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
	puts("Use arroe keys and WASD to simulate joysticks\nUse 'g' to select goal mode (auto)\nUse 'm' to select manual mode");
	puts("Also 'e' is a simulated button");
	
	for(;;)
	{
		if(read(kfd, &c, 1) < 0)
		{
			perror("read():");
			exit(-1);
		}

		linearVel = angularVel = 0;
		manMode = autMode = 0;
		
		sensor_msgs::Joy msg;
		msg.axes.resize(4);
		msg.buttons.resize(3);
		msg.axes[0] = 0.0;
		msg.axes[1] = linearVel;
		msg.axes[2] = 0.0;
		msg.axes[3] = angularVel;
		msg.buttons[0] = manMode;
		msg.buttons[1] = autMode;
		msg.buttons[2] = 0;

		switch(c)
		{
			case KEYCODE_UP:
				linearVel = 1.0;
				msg.axes[1] = linearVel;
				dirty = true;
				break;
			case KEYCODE_DOWN:
				linearVel = -1.0;
				msg.axes[1] = linearVel;
				dirty = true;
				break;
			case KEYCODE_LEFT:
				angularVel = 1.0;
				msg.axes[3] = angularVel;
				dirty = true;
				break;
			case KEYCODE_RIGHT:
				angularVel = -1.0;
				msg.axes[3] = angularVel;
				dirty = true;
				break;
			case 'a':
				msg.axes[2] = 1.0;
				dirty = true;
				break;
			case 'd':
				msg.axes[2] = -1.0;
				dirty = true;
				break;
			case 'w':
				msg.axes[0] = 1.0;
				dirty = true;
				break;
			case 's':
				msg.axes[0] = -1.0;
				dirty = true;
				break;
			case 'm':
				manMode = 1;
				msg.buttons[0] = manMode;
				puts("manual mode selected");
				dirty = true;
				break;
			case 'g':
				autMode = 1;
				msg.buttons[1] = autMode;
				puts("auto mode selected");
				dirty = true;
				break;
			case 'e':
				msg.buttons[2] = 1;
				dirty = true;
				break;
		}

		if(dirty == true)
		{
			joyPub.publish(msg);
			dirty = false;
			usleep(500000);
			msg.axes.resize(4);
			msg.buttons.resize(3);
			msg.axes[0] = 0;
			msg.axes[1] = 0;
			msg.axes[2] = 0;
			msg.axes[3] = 0;
			msg.buttons[0] = 0;
			msg.buttons[1] = 0;
			msg.buttons[2] = 0;
			joyPub.publish(msg);
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
