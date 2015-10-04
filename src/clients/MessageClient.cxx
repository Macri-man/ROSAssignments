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
  ros::ServiceClient concateclient = n.serviceClient<assignment1::Messager>("concate");
  ros::Rate loop_rate(1);
  std_msgs::String msg;
  std_msgs::String received_msg;
  std::stringstream ss;
  int state=0;
  while(ros::ok()){

    if (commandclient.call(srv)){
        received_msg.data=srv.response.command;
        ROS_INFO("Command Received: %s", srv.response.command.c_str());
        state=srv.response.state;
    }else{
      ROS_ERROR("Failed to call service command");
      return 1;
    }
    ss << "hello world " << count;
    srv.request.reqchatter = ss.str();
    srv.request.reqconcate = ss.str();
    switch(state){
      case 0:
        ROS_INFO("NO Command has been received!");
        break;
      case 1:
        ROS_INFO("Starting Talking with command: %s",received_msg.data.c_str());
        state=4;
      case 4:
        ROS_INFO("%s", srv.request.reqchatter.c_str());
        concateclient.call(srv);
        chatterclient.call(srv);
        ++count;
      break;
      case 2:
        ROS_INFO("Stopped Talking with command: %s",received_msg.data.c_str());
        state=5;
      case 5:
        ROS_INFO("%s", msg.data.c_str());
        count=0;
      break;
      case 3:
        ROS_INFO("Paused Talking with command: %s",received_msg.data.c_str());
        chatterclient.call(srv);
        state=6;
      case 6:
        ROS_INFO("%s", msg.data.c_str());
      break;
      default:
        ROS_INFO("Something terrible happened with command: %s",received_msg.data.c_str());
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}