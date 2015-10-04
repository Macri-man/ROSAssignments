#include "ros/ros.h"
#include "assignment1/Messager.h"
#include <cstdlib>
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv){
  ros::init(argc, argv, "Chatter");
  int count =0;
  int state=0;
  ros::NodeHandle n;
  ros::Rate loop_rate(1);
  ros::ServiceClient client = n.serviceClient<assignment1::Messager>("Chatter");
  ros::ServiceClient commandclient = n.serviceClient<assignment1::Messager>("Command");
  assignment1::Messager srv;
  std_msgs::String received_msg;
  std_msgs::String msg;
  std::stringstream ss;
  while (ros::ok()) {
    ss << "hello world " << count;
    msg.data = ss.str();
    if (commandclient.call(srv)){
      received_msg.data=srv.response.command;
      ROS_INFO("Command Received: %s", srv.response.command.c_str());
      state=srv.response.state;
    }else{
      ROS_ERROR("Failed to call service command");
      return 1;
    }
    switch(state){
      case 0:
        ROS_INFO("NO Message has been received!");
        break;
      case 1:
        srv.request.reqchatter = msg.data;
        ++count;
        ROS_INFO("Sent Chat to server: %s",received_msg.data.c_str());
      break;
      case 2:
        count=0;
        ROS_INFO("Stopped Chat with command: %s",received_msg.data.c_str());
      break;
      case 3:
        ROS_INFO("Paused Chat with command: %s",received_msg.data.c_str());
      break;
      default:
        ROS_INFO("Something terrible happened with command: %s",received_msg.data.c_str());
    }
    ROS_INFO("%s", msg.data.c_str());
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}