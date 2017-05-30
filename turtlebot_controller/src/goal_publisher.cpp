/*	Input : input da tastiera
*	Output : messaggio Pose
*	Logic: ad ogni esecuzione del loop del nodo, chiede un input (solo se il robot è in modalità automatica?), 
*		crea il messaggio e lo pubblica
*/

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <stdio.h>
#include <string>
#include <signal.h>
#include <termios.h>

#define NAME_OF_THIS_NODE "goal_publisher"

class ROSnode{
private:
	ros::NodeHandle Handle;
	ros::Publisher goal_pub;
	geometry_msgs::Pose goal;

public:
	void Prepare();
	void keyRun();
};

void ROSnode::Prepare(){
	goal_pub = Handle.advertise<geometry_msgs::Pose>("/goal", 20);

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

	std::string input;
	double new_x, new_y, new_w;
		
	//console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	//new line and end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);
		
	for(;;)
	{
		new_x = 0;
		new_y = 0;
		new_w = 0;

		puts("Insert new goal");

		std::cout << "Set x\n";
		std::cin >> input;
		//std::getline(std::cin, input);
		new_x = ::atof(input.c_str());

		std::cout << "Set y\n";
		std::cin >> input;
		//puts("Set y");
		//std::getline(std::cin, input);
		new_y = ::atof(input.c_str());
		
		std::cout << "Set w\n";
		std::cin >> input;
		//puts("Set y");
		//std::getline(std::cin, input);
		new_w = ::atof(input.c_str());

		do{
			std::cout << "Confirm [x: " << new_x <<", y: " << new_y <<", w: " << new_w << "] as new goal? (y/n)\n";
			std::cin >> input;
		}
		while(input!="y" && input!="n");
		if(input == "y"){
				puts("Goal confirmed");
				goal.orientation.x = new_x;
				goal.orientation.y = new_y;
				goal.orientation.z = 0;
				goal.orientation.w = new_w;
				goal_pub.publish(goal);
			}
		else	
			puts("Goal rejected");
	}
	return;
}

int main(int argc, char **argv){
	ros::init(argc, argv, NAME_OF_THIS_NODE);
	ROSnode goal_pub;
	goal_pub.Prepare();
	signal(SIGINT,quit);		
	goal_pub.keyRun();

	return(0);
}

