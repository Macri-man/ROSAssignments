#include "ros/ros.h"
#include "assignment1/Messager.h"
#include <sstream>
#include <string>
#include <cstdlib>

int count=0;
int chatcount=0;

bool concate(assignment1::Messager::Request  &req,assignment1::Messager::Response &res){
  std::stringstream ss;
  ss << req.reqchatter << " ";
  if(count%5==0){
    res.concate=ss.str();
    ROS_INFO("Sending Concatenated string to client: [%s]", res.concate.c_str());
    ss.str(std::string());
  }
  ++count;
  return true;
}

bool control(assignment1::Messager::Request  &req,assignment1::Messager::Response &res){
    if(req.reqcommand==""){
      ROS_INFO("No Command has been received!");
    }else if(!(req.reqcommand == "start" || req.reqcommand == "stop" ||  req.reqcommand =="pause")){
      ROS_INFO("Wrong Input: [%s]", req.reqcommand.c_str());
    }else{
      ROS_INFO("Command Sent: [%s]", req.reqcommand.c_str());
      res.command=req.reqcommand;
    }
  return true;
}

bool chatter(assignment1::Messager::Request  &req,assignment1::Messager::Response &res){
  res.chatter=req.reqchatter;
  ROS_INFO("request: [%s]", req.reqchatter.c_str());
  ROS_INFO("sending back response: [%s]", res.chatter.c_str());
  return true;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "message_server");
  ros::NodeHandle n;
  ros::ServiceServer conservice = n.advertiseService("concate", concate);
  ros::ServiceServer chatservice = n.advertiseService("chatter", chatter);
  ros::ServiceServer comservice = n.advertiseService("control_messages", control);
  ROS_INFO("Starting Server!");
  ros::spin();

  return 0;
}