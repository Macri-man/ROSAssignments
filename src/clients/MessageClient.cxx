#include "ros/ros.h"
#include "assignment1/Message.h"
#include <cstdlib>
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv){
  ros::init(argc, argv, "MessageClient");
  int count =0;
  ros::NodeHandle n;
  ros::ServiceClient concateclient = n.serviceClient<assignment1::Message>("concate");
  assignment1::Message concatesrv;
  ros::ServiceClient commandclient = n.serviceClient<assignment1::Message>("command");
  assignment1::Message commandsrv;
  ros::ServiceClient chatterclient = n.serviceClient<assignment1::Message>("chatter");
  assignment1::Message chattersrv;

  std_msgs::String msg;
  std::stringstream ss;
  while(ros::ok()){
    ss << "hello world " << count;
    ROS_INFO("%s", ss.str().c_str());

    if (concateclient.call(concatesrv)){
      ROS_INFO("Concate: %s", concatesrv.response.concate);
    }else{
      ROS_ERROR("Failed to call service add_two_ints");
      return 1;
    }

    if (commandclient.call(commandsrv)){
      ROS_INFO("Command: %s", commandsrv.response.command);
    }else{
      ROS_ERROR("Failed to call service add_two_ints");
      return 1;
    }

    if (chatterclient.call(chattersrv)){
      ROS_INFO("Chatting: %s", chattersrv.response.chatter);
    }else{
      ROS_ERROR("Failed to call service add_two_ints");
      return 1;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}