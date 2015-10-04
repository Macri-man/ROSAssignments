#include "ros/ros.h"
#include "assignment1/Messager.h"
#include <cstdlib>
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv){
  ros::init(argc, argv, "MessageClient");
  int count =0;
  ros::NodeHandle n;
  assignment1::Messager srv;
  ros::ServiceClient commandclient = n.serviceClient<assignment1::Messager>("control_messages");
  ros::ServiceClient chatterclient = n.serviceClient<assignment1::Messager>("chatter");
  ros::Rate loop_rate(1);
  std_msgs::String msg;
  std::stringstream ss;
  while(ros::ok()){
    ss << "hello world " << count;
    ROS_INFO("%s", ss.str().c_str());
    srv.request.reqchatter=ss.str();

    if (commandclient.call(srv)){
      ROS_INFO("Command: %s", srv.response.command.c_str());
    }else{
      ROS_ERROR("Failed to call service add_two_ints");
      return 1;
    }

    if (chatterclient.call(srv)){
      ROS_INFO("Chatting: %s", srv.response.chatter.c_str());
    }else{
      ROS_ERROR("Failed to call service add_two_ints");
      return 1;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}