#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_tf_broadcaster");
  ros::NodeHandle node;

  ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
  
  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(1.0);
  
  while (node.ok()){
    transform.setOrigin( tf::Vector3(0.2, 0.0, 0.1) );
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "base_laser"));
    rate.sleep();
  }
  return 0;
};