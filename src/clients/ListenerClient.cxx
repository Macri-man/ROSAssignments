#include "ros/ros.h"
#include "assignment1/Messager.h"
#include <cstdlib>
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv){
  ros::init(argc, argv, "Listener");
  int count =0;
  ros::NodeHandle n;
  ros::Rate loop_rate(1);
  ros::ServiceClient client = n.serviceClient<assignment1::Messager>("chatter");
  assignment1::Messager srv;
  std_msgs::String msg;
  std::stringstream ss;
  while (ros::ok()) {
    if (client.call(srv)){
      ROS_INFO("I heard: %s", srv.response.chatter.c_str());
    }else{
      ROS_ERROR("Failed to call service chatter");
      return 1;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}