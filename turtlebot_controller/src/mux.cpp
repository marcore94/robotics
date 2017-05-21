#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlebot_controller/SetMode.h"


#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "mux"

//-----------------------------------------------------------------
//-----------------------------------------------------------------

class ROSnode {
private: 
    ros::NodeHandle Handle;
    ros::Subscriber joySub;
    ros::Subscriber autoSub;
    ros::Publisher cmdPub;
    ros::ServiceServer modeSr;
    geometry_msgs::Twist command;
    int mode;
        
    void joyCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void autoCallback(const geometry_msgs::Twist::ConstPtr& msg);
    bool modeService(turtlebot_controller::SetMode::Request &req, turtlebot_controller::SetMode::Response &res);
public:    
    void Prepare();
    void RunContinuously();
    void RunPeriodically();
};

//-----------------------------------------------------------------
//-----------------------------------------------------------------

void ROSnode::Prepare() {
    joySub = Handle.subscribe("cmd_joy", 10, &ROSnode::joyCallback, this);    
    autoSub = Handle.subscribe("cmd_auto", 10, &ROSnode::autoCallback, this);
    modeSr = Handle.advertiseService("mode", &ROSnode::modeService, this);
    cmdPub = Handle.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 10);
	//cmdPub = Handle.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    
    mode = 0;
    
    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void ROSnode::RunContinuously() {
  ROS_INFO("Node %s running continuously.", ros::this_node::getName().c_str());
   
  ros::spin();
}

void ROSnode::RunPeriodically() {
    ROS_INFO("Node %s running periodically.", ros::this_node::getName().c_str());
    
    ros::Rate r(10); //10 hz
    while (ros::ok()) {
        cmdPub.publish(command);
        ros::spinOnce();
        r.sleep();
    }
}

bool ROSnode::modeService(turtlebot_controller::SetMode::Request &req, turtlebot_controller::SetMode::Response &res) {
	puts("a");
    mode = req.mode;
    res.ok = true;
	if(mode == 0)
		puts("mode set to manual");
	else
		puts("mode set to auto");
    
    return true;
}

void ROSnode::joyCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if(mode == 0) {
        geometry_msgs::Twist out;
        out.linear.x = msg->linear.x;
        out.angular.z = msg->angular.z;
        cmdPub.publish(out);
    }
}

void ROSnode::autoCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if(mode == 1) {
        geometry_msgs::Twist out;
        out.linear.x = msg->linear.x;
        out.angular.z = msg->angular.z;
        cmdPub.publish(out);
    }
}

//-----------------------------------------------------------------
//-----------------------------------------------------------------

int main(int argc, char **argv) {
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  ROSnode mNode;
   
  mNode.Prepare();
  mNode.RunContinuously();
  //mNode.RunPeriodically();
  
  return (0);
}


