#include "ros/ros.h"
#include "assignment1/Messager.h"
#include <cstdlib>
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv){
  ros::init(argc, argv, "MessageClient");
  int count =0;
  ros::Rate loop_rate(1);
  ros::NodeHandle n;
  assignment1::Messager srv;

  ros::ServiceClient concateclient = n.serviceClient<assignment1::Messager>("concate");
  assignment1::Messager concatesrv;
  ros::ServiceClient commandclient = n.serviceClient<assignment1::Messager>("command");
  assignment1::Messager commandsrv;
  ros::ServiceClient chatterclient = n.serviceClient<assignment1::Messager>("chatter");
  assignment1::Messager chattersrv;

  std_msgs::String msg;
  std::stringstream ss;
  while(ros::ok()){
    ss << "hello world " << count;
    ROS_INFO("%s", ss.str().c_str());
    srv.request.reqchatter=ss.str();
    srv.request.reqconcate=ss.str();
    if (concateclient.call(srv)){
      ROS_INFO("Concate: %s", srv.response.concate.c_str());
    }else{
      ROS_ERROR("Failed to call service add_two_ints");
      return 1;
    }

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